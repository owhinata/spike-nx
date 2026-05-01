/****************************************************************************
 * apps/btsensor/btsensor_cmd.h
 *
 * PC -> Hub ASCII command parser for btsensor (Issue #56 Commit D).
 *
 * Commands are one line per request, terminated by '\n' (CR ignored).
 * Replies (`OK\n` / `ERR <reason>\n`) go through the btsensor_tx
 * arbiter so they always make it through ahead of telemetry.  Lines
 * longer than BTSENSOR_CMD_MAX_LINE bytes are dropped with
 * `ERR overflow\n`.
 *
 *   IMU ON | OFF            -> toggle IMU streaming (BUNDLE wire)
 *   SENSOR ON | OFF         -> toggle LEGO sensor TLV streaming
 *   SET ODR <hz>            -> set ODR (only while IMU OFF, must be <=833)
 *   SET ACCEL_FSR <g>       -> accel FSR g (only while IMU OFF)
 *   SET GYRO_FSR <dps>      -> gyro FSR dps (only while IMU OFF)
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_BTSENSOR_CMD_H
#define __APPS_BTSENSOR_BTSENSOR_CMD_H

#include <stddef.h>
#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

#define BTSENSOR_CMD_MAX_LINE   64

/* Initialise the line buffer.  Idempotent. */

void btsensor_cmd_init(void);

/* Feed bytes received over RFCOMM.  Splits on '\n', dispatches each
 * line, queues a reply via btsensor_tx_enqueue_response().  Must run
 * on the BTstack main thread (the spp_packet_handler does).
 */

void btsensor_cmd_feed(const uint8_t *data, uint16_t len);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_BTSENSOR_CMD_H */
