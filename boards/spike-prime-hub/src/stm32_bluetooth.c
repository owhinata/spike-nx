/****************************************************************************
 * boards/spike-prime-hub/src/stm32_bluetooth.c
 *
 * CC2564C Bluetooth bring-up driver for SPIKE Prime Hub.
 *
 * High-level flow orchestrated by stm32_bluetooth_initialize() (see
 * docs/{ja,en}/drivers/bluetooth.md for the full rationale):
 *
 *   1. Start the 32.768 kHz TIM8 CH4 slow clock on PC9 and hold the chip
 *      in reset (nSHUTD LOW).  The clock must be stable before we raise
 *      nSHUTD or the CC2564C fails to leave ROM boot.
 *   2. Release nSHUTD after >= 50 ms and wait for the chip to finish
 *      booting (~150 ms).
 *   3. Hand the lower-half a fixed sequence of HCI vendor-specific
 *      commands from the TI-licensed init script (cc256x_init_script[]),
 *      verifying Command Complete (EVT 0x0E) after each.
 *   4. Send HCI_VS_Update_UART_HCI_Baud_Rate (opcode 0xFF36), wait for
 *      its Command Complete at the old rate, then call lower->setbaud()
 *      to switch both ends to the target rate (3 Mbps nominal).
 *   5. btuart_register() the lower-half so NuttX's generic upper-half
 *      can take it over and expose it through CONFIG_NET_BLUETOOTH.
 *
 * Implementation status (Issue #47 Step E — complete):
 *   E.1: skeleton + TIM8 slow clock start + nSHUTD reset sequence.
 *   E.2: init-script streaming + per-chunk Command Complete verification
 *        + baud switch (HCI_VS_Update_UART_HCI_Baud_Rate 0xFF36).
 *   E.3: btuart_register integration and the stm32_bringup hook + NVIC
 *        step 8 (USART2 + DMA1 S6/S7 at 0xA0).
 *   E.4: defconfig BT stack enable (DRIVERS_WIRELESS / DRIVERS_BLUETOOTH
 *        / WIRELESS_BLUETOOTH / BLUETOOTH_UART_OTHER / NET_BLUETOOTH).
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "spike_prime_hub.h"
#include "cc256x_init_script.h"

#ifdef CONFIG_STM32_USART2

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TI datasheet recommends >= 5 ms nSHUTD low to guarantee a full reset;
 * we use 50 ms to be safely conservative under NuttX scheduling jitter.
 */

#define BT_NSHUTD_LOW_MS       50

/* Time the CC2564C takes to finish its ROM boot after nSHUTD goes HIGH
 * before the first HCI command can be sent.  pybricks does not codify a
 * single number; 150 ms is generous and matches what btstack's CC256x
 * bring-up uses in polled ports.
 */

#define BT_BOOT_SETTLE_MS      150

/* HCI framing constants.  The cc256x init script is a concatenation of
 * H4-framed HCI_Command packets terminated by a single zero byte.
 *   byte 0 : H4 CMD prefix (always 0x01, used as the script terminator
 *            when zero)
 *   byte 1 : opcode low
 *   byte 2 : opcode high
 *   byte 3 : parameter length N
 *   byte 4..4+N-1 : parameters
 */

#define H4_CMD                 0x01
#define H4_EVT                 0x04
#define HCI_EVT_COMMAND_COMPL  0x0e

/* Standard HCI opcodes used during bring-up. */

#define HCI_OP_RESET           0x0c03  /* HCI_Reset, OGF=3 OCF=0x03 */

/* Vendor-specific opcode to change the HCI UART baud rate.
 * Parameters: little-endian uint32_t baud rate.
 */

#define HCI_VS_UPDATE_BAUDRATE 0xff36

/* Target BAUD rate after init script load.  3 Mbps is exact on PCLK1
 * = 48 MHz (USARTDIV = 1.0).  Fall back to 921,600 by redefining this
 * if real-world testing surfaces overrun / corruption.
 */

#define BT_TARGET_BAUD         3000000

/* Short pause between "chip acknowledged baud change" and "we actually
 * change BRR".  The CC2564C needs a few hundred microseconds to flush
 * its own FIFO at the old rate before switching; 5 ms is generous.
 */

#define BT_BAUD_SWITCH_MS      5

/* RX poll interval while waiting for a small HCI event during bring-up
 * (before any NuttX scheduling of the upper-half work thread is in
 * play).  Events are tiny so we can afford to sleep between polls.
 */

#define BT_RX_POLL_US          1000

/* Max time we'll wait for any single Command Complete.  CC2564C
 * responses to init-script commands are typically <10 ms.
 */

#define BT_CMD_TIMEOUT_US      (500 * 1000)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Blocking wrapper around the non-greedy lower->read().  Returns 0 on
 * success, -ETIMEDOUT if the response did not arrive within the budget.
 */

static int bt_recv_full(FAR const struct btuart_lowerhalf_s *lower,
                        FAR uint8_t *buf, size_t count)
{
  size_t  done = 0;
  int     waited_us = 0;

  while (done < count)
    {
      ssize_t n = lower->read(lower, buf + done, count - done);
      if (n > 0)
        {
          done      += (size_t)n;
          waited_us  = 0;
          continue;
        }

      if (waited_us >= BT_CMD_TIMEOUT_US)
        {
          return -ETIMEDOUT;
        }

      up_udelay(BT_RX_POLL_US);
      waited_us += BT_RX_POLL_US;
    }

  return OK;
}

/* Wait for a Command Complete for `opcode` at the current BAUD.  Any
 * unrelated HCI event that arrives first is silently discarded so the
 * CC2564C's boot-time Hardware_Error / whatever does not wedge us.
 */

static int bt_wait_command_complete(FAR const struct btuart_lowerhalf_s
                                    *lower, uint16_t opcode)
{
  uint8_t hdr[3];

  for (;;)
    {
      int ret = bt_recv_full(lower, hdr, 1);
      if (ret < 0)
        {
          return ret;
        }

      if (hdr[0] != H4_EVT)
        {
          return -EPROTO;
        }

      ret = bt_recv_full(lower, &hdr[1], 2);
      if (ret < 0)
        {
          return ret;
        }

      /* hdr[1] = Event Code, hdr[2] = parameter length.  Consume the
       * event body into a throwaway buffer and keep looping unless it
       * is the Command Complete for our opcode.
       */

      uint8_t  evt_code = hdr[1];
      uint8_t  plen     = hdr[2];
      uint8_t  body[260];

      if (plen > sizeof(body))
        {
          return -EOVERFLOW;
        }

      ret = bt_recv_full(lower, body, plen);
      if (ret < 0)
        {
          return ret;
        }

      if (evt_code != HCI_EVT_COMMAND_COMPL || plen < 4)
        {
          continue;  /* Skip non-Command-Complete events */
        }

      /* Command Complete body: [num_hci_cmd_pkts, opcode_lo, opcode_hi,
       * status, ...].
       */

      uint16_t evt_opcode = (uint16_t)body[1] | ((uint16_t)body[2] << 8);
      if (evt_opcode != opcode)
        {
          continue;  /* Different command's Complete — keep waiting */
        }

      uint8_t status = body[3];
      if (status != 0x00)
        {
          syslog(LOG_ERR, "BT: opcode 0x%04x failed, status=0x%02x\n",
                 opcode, status);
          return -EIO;
        }

      return OK;
    }
}

/* Send a parameter-less HCI command (H4_CMD + opcode + plen=0) and wait
 * for its Command Complete.  Used for HCI_Reset during bring-up.
 */

static int bt_send_simple_cmd(FAR const struct btuart_lowerhalf_s *lower,
                              uint16_t opcode)
{
  uint8_t cmd[4];
  ssize_t w;

  cmd[0] = H4_CMD;
  cmd[1] = (uint8_t)(opcode & 0xff);
  cmd[2] = (uint8_t)(opcode >> 8);
  cmd[3] = 0x00;

  w = lower->write(lower, cmd, sizeof(cmd));
  if (w < 0)
    {
      return (int)w;
    }

  return bt_wait_command_complete(lower, opcode);
}

/* Stream the entire cc256x_init_script[] blob, one H4 command per
 * iteration, blocking for a Command Complete after each.
 */

static int bt_load_init_script(FAR const struct btuart_lowerhalf_s *lower)
{
  FAR const uint8_t *data = cc256x_init_script;
  uint8_t h4_cmd = H4_CMD;

  /* The blob is a sequence of: [H4_CMD, op_lo, op_hi, plen, payload...]
   * terminated by a zero byte.
   */

  while (*data != 0x00)
    {
      if (*data != H4_CMD)
        {
          syslog(LOG_ERR, "BT: init script desync at offset %u (0x%02x)\n",
                 (unsigned)(data - cc256x_init_script), *data);
          return -EPROTO;
        }

      data++;  /* Past H4_CMD prefix */

      uint16_t opcode = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
      uint8_t  plen   = data[2];
      size_t   total  = (size_t)plen + 3;  /* opcode(2) + plen(1) */
      ssize_t  w;

      /* TX: rewrite the H4 prefix byte separately so we don't have to
       * copy the payload out of ROM-resident rodata.
       */

      w = lower->write(lower, &h4_cmd, 1);
      if (w < 0)
        {
          return (int)w;
        }

      w = lower->write(lower, data, total);
      if (w < 0)
        {
          return (int)w;
        }

      int ret = bt_wait_command_complete(lower, opcode);
      if (ret < 0)
        {
          syslog(LOG_ERR, "BT: CC for opcode 0x%04x failed: %d\n",
                 opcode, ret);
          return ret;
        }

      data += total;
    }

  return OK;
}

/* Negotiate the target BAUD rate: send HCI_VS_Update_UART_HCI_Baud_Rate
 * at the old rate, wait for Command Complete, then lower->setbaud().
 */

static int bt_negotiate_baud(FAR const struct btuart_lowerhalf_s *lower,
                             uint32_t target_baud)
{
  uint8_t cmd[8];
  ssize_t w;
  int     ret;

  cmd[0] = H4_CMD;
  cmd[1] = (uint8_t)(HCI_VS_UPDATE_BAUDRATE & 0xff);  /* opcode_lo */
  cmd[2] = (uint8_t)(HCI_VS_UPDATE_BAUDRATE >> 8);    /* opcode_hi */
  cmd[3] = 0x04;                                      /* param len */
  cmd[4] = (uint8_t)(target_baud & 0xff);
  cmd[5] = (uint8_t)((target_baud >> 8) & 0xff);
  cmd[6] = (uint8_t)((target_baud >> 16) & 0xff);
  cmd[7] = (uint8_t)((target_baud >> 24) & 0xff);

  w = lower->write(lower, cmd, sizeof(cmd));
  if (w < 0)
    {
      return (int)w;
    }

  ret = bt_wait_command_complete(lower, HCI_VS_UPDATE_BAUDRATE);
  if (ret < 0)
    {
      return ret;
    }

  /* Switch the local UART immediately on Command Complete, matching
   * pybricks/btstack which also have no post-CC delay.  The chip has
   * already finished shifting out the CC at the old rate by the time
   * bt_wait_command_complete returned.
   */

  return lower->setbaud(lower, target_baud);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bluetooth_initialize
 *
 * Description:
 *   Orchestrate CC2564C bring-up: supply the 32.768 kHz slow clock,
 *   toggle nSHUTD, stream the TI init script, negotiate the HCI baud
 *   rate and register the resulting UART driver with NuttX's BT stack.
 *
 *   Safe to call from stm32_bringup() (BOARD_LATE_INITIALIZE path).  On
 *   failure the CC2564C is left in reset (nSHUTD low) so further retries
 *   can recover without a board reset.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_bluetooth_initialize(void)
{
  FAR struct btuart_lowerhalf_s *lower;
  int ret;

  /* 1. Configure nSHUTD as output and drive it LOW immediately.  We do
   *    this *before* starting the slow clock so the chip sees a clean
   *    reset regardless of whatever state the pin was in at boot.
   */

  stm32_configgpio(GPIO_BT_NSHUTD);
  stm32_gpiowrite(GPIO_BT_NSHUTD, false);

  /* 2. Bring up the TIM8 CH4 32.768 kHz sleep clock on PC9.  This is the
   *    first thing the chip needs once it comes out of reset.
   */

  ret = stm32_bt_slowclk_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "BT: slow clock init failed: %d\n", ret);
      return ret;
    }

  /* 3. Prepare our USART2 lower-half before releasing reset so the UART
   *    is already accepting traffic the moment the chip starts talking.
   */

  lower = stm32_btuart_instantiate();
  if (lower == NULL)
    {
      syslog(LOG_ERR, "BT: btuart instantiate failed\n");
      return -ENODEV;
    }

  /* 4. Hold the reset long enough for the chip to actually reset, then
   *    release it and wait for ROM boot to finish.
   */

  up_mdelay(BT_NSHUTD_LOW_MS);
  stm32_gpiowrite(GPIO_BT_NSHUTD, true);
  up_mdelay(BT_BOOT_SETTLE_MS);

  /* Bring-up follows pybricks/btstack's canonical order (see
   * pybricks/lib/btstack/src/hci.h L744 enum):
   *
   *   5. HCI_Reset                 @ initial 115200
   *   6. HCI_VS_Update_UART_Baud   @ initial 115200 + local UART switch
   *   7. init script (custom_init) @ target 3 Mbps
   *
   * Our earlier attempt to load the init script at 115200 before the
   * baud change caused the chip to emit a Hardware Error
   * (Event_Not_Served_Time_Out, code 0x06) right after the baud switch
   * and drop any subsequent HCI_Reset.
   */

  ret = bt_send_simple_cmd(lower, HCI_OP_RESET);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BT: initial HCI_Reset failed: %d\n", ret);
      goto err;
    }

  ret = bt_negotiate_baud(lower, BT_TARGET_BAUD);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BT: baud negotiation failed: %d\n", ret);
      goto err;
    }

  ret = bt_load_init_script(lower);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BT: init script load failed: %d\n", ret);
      goto err;
    }

  syslog(LOG_INFO, "BT: CC2564C ready at %u bps\n",
         (unsigned)BT_TARGET_BAUD);

#ifdef CONFIG_BLUETOOTH_UART
  /* 7. Hand the lower-half to NuttX's generic upper-half.  btuart_register
   *    internally calls btuart_create() (from bt_uart_generic.c) and then
   *    bt_driver_register(), which — with CONFIG_NET_BLUETOOTH — ends up
   *    exposing the device to the Bluetooth subsystem so btsak can open
   *    a PF_BLUETOOTH socket against it.
   */

  ret = btuart_register(lower);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BT: btuart_register failed: %d\n", ret);
      goto err;
    }
#else
#  warning "CONFIG_BLUETOOTH_UART=n: BT hardware is ready but not exposed \
to the NuttX HCI stack"
#endif

  return OK;

err:
  /* Park the chip back in reset so a later retry starts clean. */

  stm32_gpiowrite(GPIO_BT_NSHUTD, false);
  return ret;
}

#endif /* CONFIG_STM32_USART2 */
