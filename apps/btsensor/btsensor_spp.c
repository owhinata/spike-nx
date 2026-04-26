/****************************************************************************
 * apps/btsensor/btsensor_spp.c
 *
 * SPIKE Prime Hub SPP server.
 *
 * Mirrors btstack's spp_counter example: L2CAP + RFCOMM + SDP stack,
 * SSP Just-Works pairing, fixed RFCOMM server channel announced over SDP
 * as "SPIKE IMU Stream".  Issue #56 Commit B changes the default GAP
 * posture to "discoverable+connectable OFF" so the daemon is silent at
 * boot; the BT button (Commit C) toggles advertising on demand.
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>

#include "bluetooth.h"
#include "btstack_event.h"
#include "btstack_util.h"
#include "classic/rfcomm.h"
#include "classic/sdp_server.h"
#include "classic/sdp_util.h"
#include "classic/spp_server.h"
#include "gap.h"
#include "hci.h"

#include "btsensor_spp.h"
#include "btsensor_tx.h"
#include "imu_sampler.h"

/* Defined in btsensor_main.c — let the daemon's teardown FSM observe
 * RFCOMM open/close.  Forward-declared instead of pulled into a public
 * header to keep the spp/main split intact.
 */

void btsensor_set_rfcomm_cid(uint16_t cid);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 0x001F00 = Uncategorized "misc" device.  Good enough for a generic SPP
 * endpoint; macOS / BlueZ will just list it as "Uncategorized".
 */

#define BTSENSOR_CLASS_OF_DEVICE   0x001F00

/****************************************************************************
 * Private Data
 ****************************************************************************/

static btstack_packet_callback_registration_t g_hci_event_cb;
static uint8_t  g_sdp_record_buffer[150];
static uint16_t g_rfcomm_cid;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void spp_packet_handler(uint8_t packet_type, uint16_t channel,
                               uint8_t *packet, uint16_t size)
{
  (void)channel;

  bd_addr_t addr;

  switch (packet_type)
    {
      case HCI_EVENT_PACKET:
        switch (hci_event_packet_get_type(packet))
          {
            case HCI_EVENT_PIN_CODE_REQUEST:
              hci_event_pin_code_request_get_bd_addr(packet, addr);
              syslog(LOG_INFO,
                     "btsensor: legacy pairing with %s, replying 0000\n",
                     bd_addr_to_str(addr));
              gap_pin_code_response(addr, "0000");
              break;

            case HCI_EVENT_USER_CONFIRMATION_REQUEST:
              syslog(LOG_INFO,
                     "btsensor: SSP confirm (numeric %06" PRIu32
                     ") — auto accept\n",
                     little_endian_read_32(packet, 8));
              break;

            case HCI_EVENT_SIMPLE_PAIRING_COMPLETE:
              hci_event_simple_pairing_complete_get_bd_addr(packet, addr);
              syslog(LOG_INFO,
                     "btsensor: SSP pairing with %s status 0x%02x\n",
                     bd_addr_to_str(addr),
                     hci_event_simple_pairing_complete_get_status(packet));
              break;

            case RFCOMM_EVENT_INCOMING_CONNECTION:
              rfcomm_event_incoming_connection_get_bd_addr(packet, addr);
              g_rfcomm_cid =
                  rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
              syslog(LOG_INFO,
                     "btsensor: RFCOMM incoming cid=%u from %s\n",
                     g_rfcomm_cid, bd_addr_to_str(addr));
              rfcomm_accept_connection(g_rfcomm_cid);
              break;

            case RFCOMM_EVENT_CHANNEL_OPENED:
              if (rfcomm_event_channel_opened_get_status(packet) != 0)
                {
                  syslog(LOG_WARNING,
                         "btsensor: RFCOMM open failed status 0x%02x\n",
                         rfcomm_event_channel_opened_get_status(packet));
                  g_rfcomm_cid = 0;
                  imu_sampler_set_rfcomm_cid(0, 0);
                  btsensor_set_rfcomm_cid(0);
                }
              else
                {
                  g_rfcomm_cid =
                      rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                  uint16_t mtu =
                      rfcomm_event_channel_opened_get_max_frame_size(packet);
                  syslog(LOG_INFO, "btsensor: RFCOMM open cid=%u mtu=%u\n",
                         g_rfcomm_cid, mtu);
                  imu_sampler_set_rfcomm_cid(g_rfcomm_cid, mtu);
                  btsensor_set_rfcomm_cid(g_rfcomm_cid);
                }
              break;

            case RFCOMM_EVENT_CHANNEL_CLOSED:
              syslog(LOG_INFO, "btsensor: RFCOMM closed cid=%u\n",
                     g_rfcomm_cid);
              g_rfcomm_cid = 0;
              imu_sampler_set_rfcomm_cid(0, 0);
              btsensor_set_rfcomm_cid(0);
              break;

            case RFCOMM_EVENT_CAN_SEND_NOW:
              btsensor_tx_on_can_send_now();
              break;

            default:
              break;
          }
        break;

      case RFCOMM_DATA_PACKET:
        /* Commit D wires the ASCII command parser here. */

        syslog(LOG_DEBUG, "btsensor: RFCOMM rx %u bytes\n", size);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void spp_server_init(void)
{
  /* Hook HCI events (pairing, RFCOMM lifecycle). */

  g_hci_event_cb.callback = &spp_packet_handler;
  hci_add_event_handler(&g_hci_event_cb);

  /* Bring up the Classic BT protocol stack. */

  l2cap_init();
  rfcomm_init();
  rfcomm_register_service(&spp_packet_handler, BTSENSOR_SPP_RFCOMM_CHANNEL,
                          0xffff);

  /* Build and register the SPP SDP record. */

  sdp_init();
  memset(g_sdp_record_buffer, 0, sizeof(g_sdp_record_buffer));
  spp_create_sdp_record(g_sdp_record_buffer,
                        sdp_create_service_record_handle(),
                        BTSENSOR_SPP_RFCOMM_CHANNEL,
                        BTSENSOR_SPP_SERVICE_NAME);
  sdp_register_service(g_sdp_record_buffer);

  /* GAP: device identity + SSP Just-Works pairing.  Discoverable and
   * connectable stay OFF — the BT button (Commit C) flips them on.
   */

  gap_set_class_of_device(BTSENSOR_CLASS_OF_DEVICE);
  gap_set_local_name(BTSENSOR_SPP_LOCAL_NAME);
  gap_discoverable_control(0);
  gap_connectable_control(0);
  gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
  gap_set_security_level(LEVEL_2);
}
