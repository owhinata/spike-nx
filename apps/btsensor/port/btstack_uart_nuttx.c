/****************************************************************************
 * apps/btsensor/port/btstack_uart_nuttx.c
 *
 * btstack_uart_t wrapper around the /dev/ttyBT character device provided by
 * boards/spike-prime-hub (Issue #52 Step C).  Non-blocking read/write are
 * driven by the run loop's poll(2) loop via btstack_data_source_t.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <arch/board/board_btuart.h>

#include "btstack_debug.h"
#include "btstack_run_loop.h"
#include "btstack_uart.h"

#include "btstack_uart_nuttx.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const btstack_uart_config_t *g_uart_config;

static btstack_data_source_t g_ds;

static void (*g_block_sent)(void);
static void (*g_block_received)(void);

static const uint8_t *g_tx_data;
static uint16_t       g_tx_len;

static uint8_t *g_rx_data;
static uint16_t g_rx_len;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void uart_nuttx_process(btstack_data_source_t *ds,
                               btstack_data_source_callback_type_t type)
{
  if (type == DATA_SOURCE_CALLBACK_READ && g_rx_len > 0)
    {
      ssize_t n = read(ds->source.fd, g_rx_data, g_rx_len);
      if (n > 0)
        {
          g_rx_data += n;
          g_rx_len  -= n;
        }
      else if (n < 0 && errno != EAGAIN && errno != EINTR)
        {
          log_error("btstack_uart_nuttx: read errno %d", errno);
          btstack_run_loop_disable_data_source_callbacks(
              ds, DATA_SOURCE_CALLBACK_READ);
          return;
        }

      if (g_rx_len == 0)
        {
          btstack_run_loop_disable_data_source_callbacks(
              ds, DATA_SOURCE_CALLBACK_READ);
          if (g_block_received)
            {
              g_block_received();
            }
        }
    }

  if (type == DATA_SOURCE_CALLBACK_WRITE && g_tx_len > 0)
    {
      ssize_t n = write(ds->source.fd, g_tx_data, g_tx_len);
      if (n > 0)
        {
          g_tx_data += n;
          g_tx_len  -= n;
        }
      else if (n < 0 && errno != EAGAIN && errno != EINTR)
        {
          log_error("btstack_uart_nuttx: write errno %d", errno);
          btstack_run_loop_disable_data_source_callbacks(
              ds, DATA_SOURCE_CALLBACK_WRITE);
          return;
        }

      if (g_tx_len == 0)
        {
          btstack_run_loop_disable_data_source_callbacks(
              ds, DATA_SOURCE_CALLBACK_WRITE);
          if (g_block_sent)
            {
              g_block_sent();
            }
        }
    }
}

/****************************************************************************
 * btstack_uart_t callbacks
 ****************************************************************************/

static int uart_nuttx_init(const btstack_uart_config_t *config)
{
  g_uart_config = config;
  return 0;
}

static int uart_nuttx_open(void)
{
  int fd = open(BOARD_BTUART_DEVPATH, O_RDWR | O_NONBLOCK);
  if (fd < 0)
    {
      log_error("btstack_uart_nuttx: open %s errno %d",
                BOARD_BTUART_DEVPATH, errno);
      return -1;
    }

  btstack_run_loop_set_data_source_fd(&g_ds, fd);
  btstack_run_loop_set_data_source_handler(&g_ds, uart_nuttx_process);
  btstack_run_loop_add_data_source(&g_ds);

  /* Apply the initial baud requested by hci.c even though our chardev was
   * opened at whatever the kernel driver defaulted to.  This matters after
   * the HCI baud switch during init-script streaming.
   */

  if (g_uart_config != NULL && g_uart_config->baudrate != 0)
    {
      if (ioctl(fd, BTUART_IOC_SETBAUD,
                (unsigned long)g_uart_config->baudrate) < 0)
        {
          log_error("btstack_uart_nuttx: initial setbaud %lu errno %d",
                    (unsigned long)g_uart_config->baudrate, errno);
          /* Non-fatal: controller boots at 115200 anyway. */
        }
    }

  return 0;
}

static int uart_nuttx_close(void)
{
  int fd = g_ds.source.fd;
  btstack_run_loop_remove_data_source(&g_ds);
  if (fd >= 0)
    {
      close(fd);
    }

  g_ds.source.fd = -1;
  g_tx_len       = 0;
  g_rx_len       = 0;
  return 0;
}

static void uart_nuttx_set_block_received(void (*cb)(void))
{
  g_block_received = cb;
}

static void uart_nuttx_set_block_sent(void (*cb)(void))
{
  g_block_sent = cb;
}

static int uart_nuttx_set_baudrate(uint32_t baudrate)
{
  if (ioctl(g_ds.source.fd, BTUART_IOC_SETBAUD,
            (unsigned long)baudrate) < 0)
    {
      return -1;
    }

  return 0;
}

static int uart_nuttx_set_parity(int parity)
{
  /* HCI always runs parity off; stm32_btuart_chardev ignores the call. */

  (void)parity;
  return 0;
}

static int uart_nuttx_set_flowcontrol(int flowcontrol)
{
  /* HW flow control is always enabled on USART2 CTS/RTS. */

  (void)flowcontrol;
  return 0;
}

static void uart_nuttx_receive_block(uint8_t *buffer, uint16_t len)
{
  g_rx_data = buffer;
  g_rx_len  = len;
  btstack_run_loop_enable_data_source_callbacks(&g_ds,
                                                DATA_SOURCE_CALLBACK_READ);
}

static void uart_nuttx_send_block(const uint8_t *buffer, uint16_t len)
{
  g_tx_data = buffer;
  g_tx_len  = len;
  btstack_run_loop_enable_data_source_callbacks(&g_ds,
                                                DATA_SOURCE_CALLBACK_WRITE);
}

/****************************************************************************
 * Public API
 ****************************************************************************/

const btstack_uart_t *btstack_uart_nuttx_instance(void)
{
  static const btstack_uart_t instance =
  {
    &uart_nuttx_init,
    &uart_nuttx_open,
    &uart_nuttx_close,
    &uart_nuttx_set_block_received,
    &uart_nuttx_set_block_sent,
    &uart_nuttx_set_baudrate,
    &uart_nuttx_set_parity,
    &uart_nuttx_set_flowcontrol,
    &uart_nuttx_receive_block,
    &uart_nuttx_send_block,
    NULL,     /* get_supported_sleep_modes */
    NULL,     /* set_sleep */
    NULL,     /* set_wakeup_handler */
    NULL,     /* set_frame_received */
    NULL,     /* set_frame_sent */
    NULL,     /* receive_frame */
    NULL,     /* send_frame */
  };

  return &instance;
}
