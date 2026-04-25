/****************************************************************************
 * apps/btsensor/btsensor_spp.c
 *
 * SPIKE Prime Hub SPP server (Issue #52 Step D).
 *
 * Mirrors btstack's spp_counter example: L2CAP + RFCOMM + SDP stack,
 * SSP Just-Works pairing, fixed RFCOMM server channel announced over SDP
 * as "SPIKE IMU Stream".  The Step E sampler will hook into the RFCOMM
 * channel id captured from RFCOMM_EVENT_CHANNEL_OPENED to stream IMU
 * frames via rfcomm_send + RFCOMM_EVENT_CAN_SEND_NOW.
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

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
#include "imu_sampler.h"

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
              /* Legacy pairing: reply with "0000".  Kept as a fallback for
               * older hosts; SSP Just-Works is preferred.
               */

              hci_event_pin_code_request_get_bd_addr(packet, addr);
              printf("btsensor: legacy pairing with %s, replying with 0000\n",
                     bd_addr_to_str(addr));
              gap_pin_code_response(addr, "0000");
              break;

            case HCI_EVENT_USER_CONFIRMATION_REQUEST:
              /* SSP Just-Works: btstack auto-accepts when the IO capability
               * is DISPLAY_YES_NO and we do nothing here.  Log the numeric
               * value so the host dialog can be cross-checked.
               */

              printf("btsensor: SSP confirm (numeric %06" PRIu32 ") — auto accept\n",
                     little_endian_read_32(packet, 8));
              break;

            case HCI_EVENT_SIMPLE_PAIRING_COMPLETE:
              hci_event_simple_pairing_complete_get_bd_addr(packet, addr);
              printf("btsensor: SSP pairing with %s status 0x%02x\n",
                     bd_addr_to_str(addr),
                     hci_event_simple_pairing_complete_get_status(packet));
              break;

            case RFCOMM_EVENT_INCOMING_CONNECTION:
              rfcomm_event_incoming_connection_get_bd_addr(packet, addr);
              g_rfcomm_cid =
                  rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
              printf("btsensor: RFCOMM incoming cid=%u from %s\n",
                     g_rfcomm_cid, bd_addr_to_str(addr));
              rfcomm_accept_connection(g_rfcomm_cid);
              break;

            case RFCOMM_EVENT_CHANNEL_OPENED:
              if (rfcomm_event_channel_opened_get_status(packet) != 0)
                {
                  printf("btsensor: RFCOMM open failed status 0x%02x\n",
                         rfcomm_event_channel_opened_get_status(packet));
                  g_rfcomm_cid = 0;
                  imu_sampler_set_rfcomm_cid(0, 0);
                }
              else
                {
                  g_rfcomm_cid =
                      rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                  uint16_t mtu =
                      rfcomm_event_channel_opened_get_max_frame_size(packet);
                  printf("btsensor: RFCOMM open cid=%u mtu=%u\n",
                         g_rfcomm_cid, mtu);
                  imu_sampler_set_rfcomm_cid(g_rfcomm_cid, mtu);
                }
              break;

            case RFCOMM_EVENT_CHANNEL_CLOSED:
              printf("btsensor: RFCOMM closed cid=%u\n", g_rfcomm_cid);
              g_rfcomm_cid = 0;
              imu_sampler_set_rfcomm_cid(0, 0);
              break;

            case RFCOMM_EVENT_CAN_SEND_NOW:
              imu_sampler_on_can_send_now();
              break;

            default:
              break;
          }
        break;

      case RFCOMM_DATA_PACKET:
        /* Step D: log what the host sends so we can verify round-trip data.
         * Step E will interpret Pybricks-style rate/reset control frames.
         */

        printf("btsensor: RFCOMM rx %u bytes\n", size);
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

  /* Build and register the SPP SDP record advertising our channel number
   * + "SPIKE IMU Stream" service name.
   */

  sdp_init();
  memset(g_sdp_record_buffer, 0, sizeof(g_sdp_record_buffer));
  spp_create_sdp_record(g_sdp_record_buffer,
                        sdp_create_service_record_handle(),
                        BTSENSOR_SPP_RFCOMM_CHANNEL,
                        BTSENSOR_SPP_SERVICE_NAME);
  sdp_register_service(g_sdp_record_buffer);

  /* GAP: device identity + SSP Just-Works pairing. */

  gap_set_class_of_device(BTSENSOR_CLASS_OF_DEVICE);
  gap_set_local_name(BTSENSOR_SPP_LOCAL_NAME);
  gap_discoverable_control(1);
  gap_connectable_control(1);
  gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
  gap_set_security_level(LEVEL_2);
}
