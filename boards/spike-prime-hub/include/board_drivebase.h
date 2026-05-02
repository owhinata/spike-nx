/****************************************************************************
 * boards/spike-prime-hub/include/board_drivebase.h
 *
 * /dev/drivebase character-device ABI for the SPIKE Prime drivebase
 * daemon (Issue #77).  The kernel side is a thin shim that registers the
 * device node and translates user ioctls into envelopes consumed by a
 * userspace daemon (apps/drivebase/) over a kernel-resident SPSC command
 * ring; daemon state publish goes the other direction through a kernel-
 * resident double-buffer.  The daemon-internal ioctls (DAEMON_*) are
 * private to that bridge and disappear when the same ABI is hosted on
 * Linux/FUSE — the Linux port replaces the kernel shim with a libfuse
 * callback table that drains user-facing ioctls directly into the
 * daemon-side handler, making the user-facing ABI byte-identical across
 * the two environments.
 *
 * ABI rules (32 vs 64-bit interop / FUSE portability):
 *   - Fixed-width types only (uint32_t / int32_t / uint64_t / uint8_t).
 *     No `long`, no pointer-in-struct, no `time_t`, no `bool`.
 *   - All structs are sized & laid out with _Static_assert below.
 *   - No variable-length payloads or pointer-bearing structs (FUSE_IOCTL
 *     unrestricted ioctl semantics are hostile to either).
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_DRIVEBASE_H
#define __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_DRIVEBASE_H

#include <stddef.h>
#include <stdint.h>
#include <sys/ioctl.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions: ioctl numbers
 ****************************************************************************/

#define DRIVEBASE_DEVPATH            "/dev/drivebase"

#define _DBASEBASE                   (0x4900)
#define _DBASEIOC(nr)                _IOC(_DBASEBASE, nr)

/* User-facing ioctls.  These survive a future port to Linux/FUSE; the
 * struct layouts below are the wire ABI for both environments.
 */

/* Lifecycle / configuration */

#define DRIVEBASE_CONFIG             _DBASEIOC(0x01)  /* drivebase_config_s   */
#define DRIVEBASE_RESET              _DBASEIOC(0x02)  /* drivebase_reset_s    */

/* Drive primitives */

#define DRIVEBASE_DRIVE_STRAIGHT     _DBASEIOC(0x10)  /* drivebase_drive_straight_s */
#define DRIVEBASE_DRIVE_CURVE        _DBASEIOC(0x11)  /* drivebase_drive_curve_s    */
#define DRIVEBASE_DRIVE_ARC_ANGLE    _DBASEIOC(0x12)  /* drivebase_drive_arc_s      */
#define DRIVEBASE_DRIVE_ARC_DISTANCE _DBASEIOC(0x13)  /* drivebase_drive_arc_s      */
#define DRIVEBASE_DRIVE_FOREVER      _DBASEIOC(0x14)  /* drivebase_drive_forever_s  */
#define DRIVEBASE_TURN               _DBASEIOC(0x15)  /* drivebase_turn_s           */
#define DRIVEBASE_STOP               _DBASEIOC(0x16)  /* drivebase_stop_s           */

/* SPIKE-specific drive wrappers (pybricks `drivebase_spike_*` parity) */

#define DRIVEBASE_SPIKE_DRIVE_FOREVER _DBASEIOC(0x20) /* drivebase_spike_forever_s */
#define DRIVEBASE_SPIKE_DRIVE_TIME    _DBASEIOC(0x21) /* drivebase_spike_time_s    */
#define DRIVEBASE_SPIKE_DRIVE_ANGLE   _DBASEIOC(0x22) /* drivebase_spike_angle_s   */

/* Settings + state */

#define DRIVEBASE_GET_DRIVE_SETTINGS _DBASEIOC(0x30)  /* drivebase_drive_settings_s */
#define DRIVEBASE_SET_DRIVE_SETTINGS _DBASEIOC(0x31)  /* drivebase_drive_settings_s */
#define DRIVEBASE_SET_USE_GYRO       _DBASEIOC(0x32)  /* drivebase_set_use_gyro_s   */
#define DRIVEBASE_GET_STATE          _DBASEIOC(0x33)  /* drivebase_state_s          */
#define DRIVEBASE_GET_HEADING        _DBASEIOC(0x34)  /* drivebase_heading_s        */

/* Diagnostics */

#define DRIVEBASE_JITTER_DUMP        _DBASEIOC(0x40)  /* drivebase_jitter_dump_s    */
#define DRIVEBASE_GET_STATUS         _DBASEIOC(0x41)  /* drivebase_status_s         */

/* Daemon-internal ioctls.  Only the userspace daemon (one per board)
 * issues these; userspace clients must not touch them.  The Linux/FUSE
 * port elides this entire block — the FUSE callback table drains
 * user-facing ioctls straight into the daemon handler.
 */

#define DRIVEBASE_DAEMON_ATTACH         _DBASEIOC(0x80) /* drivebase_attach_s   */
#define DRIVEBASE_DAEMON_DETACH         _DBASEIOC(0x81) /* arg ignored          */
#define DRIVEBASE_DAEMON_PICKUP_CMD     _DBASEIOC(0x82) /* drivebase_cmd_envelope_s* */
#define DRIVEBASE_DAEMON_PUBLISH_STATE  _DBASEIOC(0x83) /* drivebase_state_s *  */
#define DRIVEBASE_DAEMON_PUBLISH_STATUS _DBASEIOC(0x84) /* drivebase_status_s * */
#define DRIVEBASE_DAEMON_PUBLISH_JITTER _DBASEIOC(0x85) /* drivebase_jitter_dump_s* */

/****************************************************************************
 * Public Types: enums
 ****************************************************************************/

/* on_completion (pybricks `pbio_control_on_completion_t` parity).  See
 * docs/{ja,en}/drivers/drivebase.md §completion-policies for the SMART
 * variants — they keep torque for `smart_passive_hold_time` after
 * `is_done` and let the next relative command continue from the prior
 * endpoint when the current position is within `position_tolerance × 2`
 * of it.
 */

enum drivebase_on_completion_e
{
  DRIVEBASE_ON_COMPLETION_COAST       = 0,
  DRIVEBASE_ON_COMPLETION_BRAKE       = 1,
  DRIVEBASE_ON_COMPLETION_HOLD        = 2,
  DRIVEBASE_ON_COMPLETION_CONTINUE    = 3,
  DRIVEBASE_ON_COMPLETION_COAST_SMART = 4,
  DRIVEBASE_ON_COMPLETION_BRAKE_SMART = 5,
};

enum drivebase_use_gyro_e
{
  DRIVEBASE_USE_GYRO_NONE = 0,
  DRIVEBASE_USE_GYRO_1D   = 1,
  DRIVEBASE_USE_GYRO_3D   = 2,
};

enum drivebase_active_command_e
{
  DRIVEBASE_ACTIVE_NONE     = 0,
  DRIVEBASE_ACTIVE_STRAIGHT = 1,
  DRIVEBASE_ACTIVE_CURVE    = 2,
  DRIVEBASE_ACTIVE_ARC      = 3,
  DRIVEBASE_ACTIVE_FOREVER  = 4,
  DRIVEBASE_ACTIVE_TURN     = 5,
  DRIVEBASE_ACTIVE_SPIKE    = 6,
  DRIVEBASE_ACTIVE_STOP     = 7,
};

/****************************************************************************
 * Public Types: user-facing structs
 ****************************************************************************/

/* Open-time configuration.  Wheel diameter / axle track only — left and
 * right port are implicit in the sensor class topic naming
 * (sensor_motor_l = odd port, sensor_motor_r = even port) so they are
 * not negotiable per-daemon-life.
 */

struct drivebase_config_s
{
  uint32_t wheel_diameter_mm;
  uint32_t axle_track_mm;
  uint8_t  reserved[8];
};

struct drivebase_reset_s
{
  int32_t distance_mm;        /* new distance origin (default 0)        */
  int32_t angle_mdeg;         /* new heading origin (default 0)         */
  uint8_t reserved[8];
};

struct drivebase_drive_straight_s
{
  int32_t distance_mm;
  uint8_t on_completion;      /* enum drivebase_on_completion_e         */
  uint8_t reserved[7];
};

struct drivebase_drive_curve_s
{
  int32_t radius_mm;
  int32_t angle_deg;
  uint8_t on_completion;
  uint8_t reserved[7];
};

struct drivebase_drive_arc_s
{
  int32_t radius_mm;
  int32_t arg;                /* ARC_ANGLE: angle_deg, ARC_DISTANCE: mm */
  uint8_t on_completion;
  uint8_t reserved[7];
};

struct drivebase_drive_forever_s
{
  int32_t speed_mmps;
  int32_t turn_rate_dps;
};

struct drivebase_turn_s
{
  int32_t angle_deg;
  uint8_t on_completion;
  uint8_t reserved[3];
};

struct drivebase_stop_s
{
  uint8_t on_completion;
  uint8_t reserved[7];
};

struct drivebase_spike_forever_s
{
  int32_t speed_left_mmps;
  int32_t speed_right_mmps;
};

struct drivebase_spike_time_s
{
  int32_t  speed_left_mmps;
  int32_t  speed_right_mmps;
  uint32_t duration_ms;
  uint8_t  on_completion;
  uint8_t  reserved[3];
};

struct drivebase_spike_angle_s
{
  int32_t speed_left_mmps;
  int32_t speed_right_mmps;
  int32_t angle_deg;
  uint8_t on_completion;
  uint8_t reserved[3];
};

struct drivebase_drive_settings_s
{
  int32_t drive_speed_mmps;
  int32_t drive_acceleration;
  int32_t drive_deceleration;
  int32_t turn_rate_dps;
  int32_t turn_acceleration;
  int32_t turn_deceleration;
};

struct drivebase_set_use_gyro_s
{
  uint8_t use_gyro;           /* enum drivebase_use_gyro_e              */
  uint8_t reserved[7];
};

/* Atomic snapshot of the daemon's published state. */

struct drivebase_state_s
{
  int32_t  distance_mm;
  int32_t  drive_speed_mmps;
  int32_t  angle_mdeg;        /* heading × 1000                         */
  int32_t  turn_rate_dps;
  uint32_t tick_seq;
  uint8_t  is_done;
  uint8_t  is_stalled;
  uint8_t  active_command;    /* enum drivebase_active_command_e        */
  uint8_t  reserved;
};

struct drivebase_heading_s
{
  int32_t angle_mdeg;
  int32_t turn_rate_dps;
};

/* Diagnostics: counts / backpressure / IPC health. */

struct drivebase_status_s
{
  uint8_t  configured;
  uint8_t  motor_l_bound;
  uint8_t  motor_r_bound;
  uint8_t  imu_present;
  uint8_t  use_gyro;
  uint8_t  daemon_attached;
  uint8_t  reserved[2];

  uint32_t tick_count;
  uint32_t tick_overrun_count;
  uint32_t tick_max_lag_us;

  uint32_t cmd_ring_depth;     /* current occupancy                      */
  uint32_t cmd_drop_count;     /* envelopes shed when ring full          */
  uint32_t last_cmd_seq;
  uint32_t last_pickup_us;     /* last DAEMON_PICKUP_CMD, monotonic       */
  uint32_t last_publish_us;    /* last DAEMON_PUBLISH_STATE, monotonic    */
  uint32_t attach_generation;  /* bumps on every ATTACH/DETACH/cleanup    */

  uint32_t encoder_drop_count; /* observed lump_sample_s.seq gaps         */
};

#define DRIVEBASE_JITTER_BUCKETS 8

struct drivebase_jitter_dump_s
{
  uint32_t total_ticks;
  uint32_t hist_us[DRIVEBASE_JITTER_BUCKETS];
                              /* <50 / 50-100 / 100-200 / 200-500 /
                               * 500-1k / 1k-2k / 2k-5k / 5k+            */
  uint32_t max_lag_us;
  uint32_t deadline_miss_count;
};

/****************************************************************************
 * Public Types: daemon-internal structs (kernel <-> daemon bridge)
 ****************************************************************************/

/* DAEMON_ATTACH payload.  The daemon registers the legoport indices it
 * has CLAIM-locked; the kernel uses them for the emergency-stop fast
 * path (STOP ioctl coasts both ports without a daemon round trip).  No
 * function pointer is registered — the kernel calls the kernel-resident
 * `stm32_legoport_pwm_coast/brake()` directly so a daemon segfault can
 * never leave a dangling pointer behind.
 */

struct drivebase_attach_s
{
  uint8_t motor_l_port_idx;   /* 0..5 (A..F)                             */
  uint8_t motor_r_port_idx;   /* 0..5 (A..F)                             */
  uint8_t default_on_completion;
                              /* applied if STOP ioctl arrives before    */
                              /* the daemon picks up the envelope        */
  uint8_t reserved[5];
};

/* Command envelope passed from the kernel ring to the daemon.  `cmd_kind`
 * is a user-facing ioctl number.  `epoch` is the output_epoch at the
 * time the kernel pushed the envelope; the daemon uses it to discard
 * any envelope whose epoch was superseded by a later STOP.
 */

#define DRIVEBASE_CMD_PAYLOAD_BYTES 24

struct drivebase_cmd_envelope_s
{
  uint32_t cmd_kind;          /* DRIVEBASE_DRIVE_STRAIGHT etc            */
  uint32_t cmd_seq;           /* monotonic, per-board                    */
  uint32_t epoch;             /* output_epoch at push time               */
  uint32_t reserved;
  uint8_t  payload[DRIVEBASE_CMD_PAYLOAD_BYTES];
                              /* large enough for the largest user-facing*/
                              /* drive struct (24 B for spike_angle / 24 */
                              /* B drive_settings)                       */
};

/****************************************************************************
 * ABI size / offset locks (32 vs 64 bit interop / FUSE portability)
 ****************************************************************************/

_Static_assert(sizeof(struct drivebase_config_s)         == 16, "drivebase_config_s ABI");
_Static_assert(sizeof(struct drivebase_reset_s)          == 16, "drivebase_reset_s ABI");
_Static_assert(sizeof(struct drivebase_drive_straight_s) == 12, "drivebase_drive_straight_s ABI");
_Static_assert(sizeof(struct drivebase_drive_curve_s)    == 16, "drivebase_drive_curve_s ABI");
_Static_assert(sizeof(struct drivebase_drive_arc_s)      == 16, "drivebase_drive_arc_s ABI");
_Static_assert(sizeof(struct drivebase_drive_forever_s)  ==  8, "drivebase_drive_forever_s ABI");
_Static_assert(sizeof(struct drivebase_turn_s)           ==  8, "drivebase_turn_s ABI");
_Static_assert(sizeof(struct drivebase_stop_s)           ==  8, "drivebase_stop_s ABI");
_Static_assert(sizeof(struct drivebase_spike_forever_s)  ==  8, "drivebase_spike_forever_s ABI");
_Static_assert(sizeof(struct drivebase_spike_time_s)     == 16, "drivebase_spike_time_s ABI");
_Static_assert(sizeof(struct drivebase_spike_angle_s)    == 16, "drivebase_spike_angle_s ABI");
_Static_assert(sizeof(struct drivebase_drive_settings_s) == 24, "drivebase_drive_settings_s ABI");
_Static_assert(sizeof(struct drivebase_set_use_gyro_s)   ==  8, "drivebase_set_use_gyro_s ABI");
_Static_assert(sizeof(struct drivebase_state_s)          == 24, "drivebase_state_s ABI");
_Static_assert(sizeof(struct drivebase_heading_s)        ==  8, "drivebase_heading_s ABI");
_Static_assert(sizeof(struct drivebase_status_s)         == 48, "drivebase_status_s ABI");
_Static_assert(sizeof(struct drivebase_jitter_dump_s)    == 44, "drivebase_jitter_dump_s ABI");
_Static_assert(sizeof(struct drivebase_attach_s)         ==  8, "drivebase_attach_s ABI");
_Static_assert(sizeof(struct drivebase_cmd_envelope_s)   == 40, "drivebase_cmd_envelope_s ABI");

_Static_assert(offsetof(struct drivebase_state_s, tick_seq) == 16,
               "drivebase_state_s.tick_seq offset");
_Static_assert(offsetof(struct drivebase_status_s, attach_generation) == 40,
               "drivebase_status_s.attach_generation offset");

#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_DRIVEBASE_H */
