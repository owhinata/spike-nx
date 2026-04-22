/****************************************************************************
 * apps/btsensor/btsensor_spp.h
 *
 * Classic Bluetooth SPP (Serial Port Profile over RFCOMM) server for the
 * SPIKE Prime Hub (Issue #52 Step D).
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_SPP_SERVER_H
#define __APPS_BTSENSOR_SPP_SERVER_H

#if defined __cplusplus
extern "C" {
#endif

#ifndef BTSENSOR_SPP_LOCAL_NAME
#  define BTSENSOR_SPP_LOCAL_NAME      "SPIKE-BT-Sensor"
#endif

#ifndef BTSENSOR_SPP_SERVICE_NAME
#  define BTSENSOR_SPP_SERVICE_NAME    "SPIKE IMU Stream"
#endif

/* Reserved SPP RFCOMM server channel (advertised in SDP record). */

#define BTSENSOR_SPP_RFCOMM_CHANNEL    1

/* Install the L2CAP + RFCOMM + SDP stack, register the SPP SDP record and
 * wire up the GAP options (class of device, discoverable, SSP Just Works).
 * Must be called before hci_power_control(HCI_POWER_ON).
 */

void spp_server_init(void);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_SPP_SERVER_H */
