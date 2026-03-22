/****************************************************************************
 * apps/imu/imu_geometry.c
 *
 * Linear algebra utilities for IMU processing.
 * Ported from pybricks geometry.c (MIT License, Pybricks Authors).
 ****************************************************************************/

#include <math.h>
#include "imu_geometry.h"

void imu_side_get_axis(imu_side_t side, uint8_t *index, int8_t *sign)
{
  *index = side & 0x03;
  *sign = (side & 0x04) ? -1 : 1;
}

imu_side_t imu_side_from_vector(imu_xyz_t *vector)
{
  float abs_max = 0;
  uint8_t axis = 0;
  bool negative = true;

  for (uint8_t i = 0; i < 3; i++)
    {
      if (vector->values[i] > abs_max)
        {
          abs_max = vector->values[i];
          negative = false;
          axis = i;
        }
      else if (-vector->values[i] > abs_max)
        {
          abs_max = -vector->values[i];
          negative = true;
          axis = i;
        }
    }

  return (imu_side_t)(axis | (negative << 2));
}

float imu_vector_norm(imu_xyz_t *input)
{
  return sqrtf(input->x * input->x +
               input->y * input->y +
               input->z * input->z);
}

int imu_vector_normalize(imu_xyz_t *input, imu_xyz_t *output)
{
  float norm = imu_vector_norm(input);
  if (norm == 0.0f)
    {
      return -1;
    }

  output->x = input->x / norm;
  output->y = input->y / norm;
  output->z = input->z / norm;
  return 0;
}

void imu_vector_cross(imu_xyz_t *a, imu_xyz_t *b, imu_xyz_t *output)
{
  output->x = a->y * b->z - a->z * b->y;
  output->y = a->z * b->x - a->x * b->z;
  output->z = a->x * b->y - a->y * b->x;
}

int imu_vector_project(imu_xyz_t *axis, imu_xyz_t *input, float *proj)
{
  imu_xyz_t unit;
  if (imu_vector_normalize(axis, &unit) < 0)
    {
      return -1;
    }

  *proj = unit.x * input->x + unit.y * input->y + unit.z * input->z;
  return 0;
}

void imu_vector_map(imu_matrix_3x3_t *map, imu_xyz_t *input,
                    imu_xyz_t *output)
{
  output->x = input->x * map->m11 + input->y * map->m12 +
              input->z * map->m13;
  output->y = input->x * map->m21 + input->y * map->m22 +
              input->z * map->m23;
  output->z = input->x * map->m31 + input->y * map->m32 +
              input->z * map->m33;
}

int imu_map_from_base_axes(imu_xyz_t *x_axis, imu_xyz_t *z_axis,
                           imu_matrix_3x3_t *map)
{
  imu_xyz_t xn;
  imu_xyz_t zn;
  imu_xyz_t yn;

  if (imu_vector_normalize(x_axis, &xn) < 0)
    {
      return -1;
    }

  if (imu_vector_normalize(z_axis, &zn) < 0)
    {
      return -1;
    }

  float ip = xn.x * zn.x + xn.y * zn.y + xn.z * zn.z;
  if (ip > 0.001f || ip < -0.001f)
    {
      return -1;
    }

  imu_vector_cross(&zn, &xn, &yn);

  map->m11 = xn.x; map->m12 = yn.x; map->m13 = zn.x;
  map->m21 = xn.y; map->m22 = yn.y; map->m23 = zn.y;
  map->m31 = xn.z; map->m32 = yn.z; map->m33 = zn.z;

  return 0;
}

void imu_quaternion_to_rotation_matrix(imu_quaternion_t *q,
                                       imu_matrix_3x3_t *r)
{
  r->m11 = 1 - 2 * (q->q2 * q->q2 + q->q3 * q->q3);
  r->m21 = 2 * (q->q1 * q->q2 + q->q3 * q->q4);
  r->m31 = 2 * (q->q1 * q->q3 - q->q2 * q->q4);
  r->m12 = 2 * (q->q1 * q->q2 - q->q3 * q->q4);
  r->m22 = 1 - 2 * (q->q1 * q->q1 + q->q3 * q->q3);
  r->m32 = 2 * (q->q2 * q->q3 + q->q1 * q->q4);
  r->m13 = 2 * (q->q1 * q->q3 + q->q2 * q->q4);
  r->m23 = 2 * (q->q2 * q->q3 - q->q1 * q->q4);
  r->m33 = 1 - 2 * (q->q1 * q->q1 + q->q2 * q->q2);
}

void imu_quaternion_from_gravity(imu_xyz_t *g, imu_quaternion_t *q)
{
  if (g->z >= 0)
    {
      q->q4 = sqrtf((g->z + 1) / 2);
      q->q1 = g->y / sqrtf(2 * (g->z + 1));
      q->q2 = -g->x / sqrtf(2 * (g->z + 1));
      q->q3 = 0;
    }
  else
    {
      q->q4 = -g->y / sqrtf(2 * (1 - g->z));
      q->q1 = -sqrtf((1 - g->z) / 2);
      q->q2 = 0;
      q->q3 = -g->x / sqrtf(2 * (1 - g->z));
    }
}

void imu_quaternion_get_rate_of_change(imu_quaternion_t *q, imu_xyz_t *w,
                                       imu_quaternion_t *dq)
{
  imu_xyz_t wr =
    {
      .x = w->x * IMU_DEG_TO_RAD,
      .y = w->y * IMU_DEG_TO_RAD,
      .z = w->z * IMU_DEG_TO_RAD,
    };

  dq->q1 = 0.5f * ( wr.z * q->q2 - wr.y * q->q3 + wr.x * q->q4);
  dq->q2 = 0.5f * (-wr.z * q->q1 + wr.x * q->q3 + wr.y * q->q4);
  dq->q3 = 0.5f * ( wr.y * q->q1 - wr.x * q->q2 + wr.z * q->q4);
  dq->q4 = 0.5f * (-wr.x * q->q1 - wr.y * q->q2 - wr.z * q->q3);
}

void imu_quaternion_normalize(imu_quaternion_t *q)
{
  float norm = sqrtf(q->q1 * q->q1 + q->q2 * q->q2 +
                     q->q3 * q->q3 + q->q4 * q->q4);
  if (norm < 0.0001f && norm > -0.0001f)
    {
      return;
    }

  q->q1 /= norm;
  q->q2 /= norm;
  q->q3 /= norm;
  q->q4 /= norm;
}

float imu_maxf(float a, float b)
{
  return a > b ? a : b;
}

float imu_absf(float a)
{
  return a < 0 ? -a : a;
}
