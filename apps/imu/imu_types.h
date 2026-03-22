/****************************************************************************
 * apps/imu/imu_types.h
 *
 * Type definitions for the IMU processing library.
 * Ported from pybricks pbio/geometry.h and pbio/imu.h.
 ****************************************************************************/

#ifndef __APPS_IMU_TYPES_H
#define __APPS_IMU_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMU_DEG_TO_RAD  0.017453293f
#define IMU_RAD_TO_DEG  57.29577951f
#define IMU_STANDARD_GRAVITY  9806.65f  /* mm/s^2 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Side of a box (hub) */

typedef enum
{
  IMU_SIDE_FRONT  = (0 << 2) | 0,  /* +X */
  IMU_SIDE_LEFT   = (0 << 2) | 1,  /* +Y */
  IMU_SIDE_TOP    = (0 << 2) | 2,  /* +Z */
  IMU_SIDE_BACK   = (1 << 2) | 0,  /* -X */
  IMU_SIDE_RIGHT  = (1 << 2) | 1,  /* -Y */
  IMU_SIDE_BOTTOM = (1 << 2) | 2,  /* -Z */
} imu_side_t;

/* Heading type */

typedef enum
{
  IMU_HEADING_NONE = 0,
  IMU_HEADING_1D,
  IMU_HEADING_3D,
} imu_heading_type_t;

/* 3D vector */

typedef union
{
  struct
    {
      float x;
      float y;
      float z;
    };
  float values[3];
} imu_xyz_t;

/* Quaternion */

typedef union
{
  struct
    {
      float q1;
      float q2;
      float q3;
      float q4;
    };
  float values[4];
} imu_quaternion_t;

/* 3x3 matrix */

typedef union
{
  struct
    {
      float m11; float m12; float m13;
      float m21; float m22; float m23;
      float m31; float m32; float m33;
    };
  float values[9];
} imu_matrix_3x3_t;

/* Persistent calibration settings flags */

typedef enum
{
  IMU_FLAG_GYRO_THRESHOLD   = (1 << 0),
  IMU_FLAG_ACCEL_THRESHOLD  = (1 << 1),
  IMU_FLAG_GYRO_BIAS        = (1 << 2),
  IMU_FLAG_GYRO_SCALE       = (1 << 3),
  IMU_FLAG_ACCEL_CALIBRATED = (1 << 4),
  IMU_FLAG_HEADING_1D       = (1 << 5),
} imu_settings_flags_t;

/* Persistent calibration settings (matches pybricks layout) */

typedef struct
{
  uint32_t  flags;
  float     gyro_stationary_threshold;    /* deg/s */
  float     accel_stationary_threshold;   /* mm/s^2 */
  imu_xyz_t gravity_pos;                  /* mm/s^2 per axis */
  imu_xyz_t gravity_neg;                  /* mm/s^2 per axis */
  imu_xyz_t angular_velocity_bias_start;  /* deg/s */
  imu_xyz_t angular_velocity_scale;       /* degrees per rotation */
  float     heading_correction_1d;        /* degrees per rotation */
} imu_settings_t;

#endif /* __APPS_IMU_TYPES_H */
