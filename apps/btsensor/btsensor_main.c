/****************************************************************************
 * apps/btsensor/btsensor_main.c
 *
 * Issue #52 btsensor entry point.
 *
 * Step C brought btstack to HCI_STATE_WORKING on /dev/ttyBT.
 * Step D adds the SPP server (L2CAP + RFCOMM + SDP) and SSP Just-Works
 * pairing so a PC can discover "SPIKE-BT-Sensor", pair and open the
 * RFCOMM channel advertised by the SDP record.  The main task stays
 * alive in the btstack run loop — invoke from NSH as `btsensor &`.
 * Step E wires sensor streaming into the RFCOMM channel.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "btstack_config.h"
#include "btstack_debug.h"
#include "btstack_event.h"
#include "btstack_memory.h"
#include "btstack_run_loop.h"
#include "hci.h"
#include "hci_transport.h"
#include "hci_transport_h4.h"
#include "btstack_chipset_cc256x.h"

/* Uncomment + re-enable platform/embedded/hci_dump_embedded_stdout.c in
 * apps/btsensor/Makefile to route btstack log_info/log_error + HCI
 * packet traces to printf for debugging host-side connection issues.
 */

/* #include "hci_dump.h"                         */
/* #include "hci_dump_embedded_stdout.h"         */

#include "btstack_run_loop_nuttx.h"
#include "btstack_uart_nuttx.h"
#include "btsensor_spp.h"
#include "imu_sampler.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Match stm32_btuart's initial rate.  btstack's chipset module issues its
 * own HCI_VS_Update_UART_Baud_Rate sequence and flips to the main_baudrate
 * below via the transport's set_baudrate callback.
 */

#define BT_INIT_BAUDRATE   115200
#define BT_MAIN_BAUDRATE   3000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const hci_transport_config_uart_t g_hci_transport_config =
{
  HCI_TRANSPORT_CONFIG_UART,
  BT_INIT_BAUDRATE,
  BT_MAIN_BAUDRATE,
  0,                          /* flowcontrol is already on in HW */
  NULL,                       /* device_name — ignored by our UART impl */
  BTSTACK_UART_PARITY_OFF,
};

static btstack_packet_callback_registration_t g_hci_event_reg;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void packet_handler(uint8_t packet_type, uint16_t channel,
                           uint8_t *packet, uint16_t size)
{
  (void)channel;
  (void)size;

  if (packet_type != HCI_EVENT_PACKET)
    {
      return;
    }

  switch (hci_event_packet_get_type(packet))
    {
      case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING)
          {
            bd_addr_t addr;
            gap_local_bd_addr(addr);
            printf("btsensor: HCI working, BD_ADDR %s — "
                   "advertising as \"" BTSENSOR_SPP_LOCAL_NAME "\"\n",
                   bd_addr_to_str(addr));
          }
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Entry Point
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  (void)argc;
  (void)argv;

  printf("btsensor: bringing up btstack on /dev/ttyBT\n");

  /* 1. Run loop. */

  btstack_memory_init();
  btstack_run_loop_init(btstack_run_loop_nuttx_get_instance());

  /* Enable the HCI/LOG dump on the console when debugging host-side
   * connection failures.  Also flip the two #include lines at the top
   * and re-enable hci_dump_embedded_stdout.c in apps/btsensor/Makefile.
   *
   * hci_dump_init(hci_dump_embedded_stdout_get_instance());
   */

  /* 2. HCI H4 transport on top of our NuttX UART wrapper. */

  const hci_transport_t *transport =
      hci_transport_h4_instance_for_uart(btstack_uart_nuttx_instance());

  hci_init(transport, &g_hci_transport_config);

  /* 3. CC2564C chipset helper handles init script streaming + baud switch
   * in response to HCI_Reset Command Complete.
   */

  hci_set_chipset(btstack_chipset_cc256x_instance());

  /* 4. Hook BTSTACK_EVENT_STATE for the WORKING banner. */

  g_hci_event_reg.callback = &packet_handler;
  hci_add_event_handler(&g_hci_event_reg);

  /* 5. Bring up the SPP protocol stack (L2CAP + RFCOMM + SDP + GAP opts)
   * before powering HCI on so the SDP record is registered in time for
   * the first inquiry from the host.
   */

  spp_server_init();

  /* 6. IMU sampler — opens uORB accel/gyro, hooks them into the run loop
   * and will start enqueueing frames as soon as RFCOMM_EVENT_CHANNEL_OPENED
   * hands us an RFCOMM cid.
   */

  if (imu_sampler_init() != 0)
    {
      printf("btsensor: imu sampler init failed, streaming disabled\n");
    }

  /* 7. Turn the radio on and enter the run loop.  hci_power_control
   * triggers the bring-up state machine (HCI_Reset → chipset init script
   * → baud switch → local name + BD_ADDR read → STATE_WORKING).  The run
   * loop is entered afterwards and never returns unless the caller posts
   * btstack_run_loop_trigger_exit() from a packet handler.
   */

  hci_power_control(HCI_POWER_ON);
  btstack_run_loop_execute();

  return 0;
}
