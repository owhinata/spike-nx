/****************************************************************************
 * apps/imu/imu_geometry.h
 *
 * Linear algebra utilities for IMU processing.
 * Ported from pybricks geometry.h/geometry.c.
 ****************************************************************************/

#ifndef __APPS_IMU_GEOMETRY_H
#define __APPS_IMU_GEOMETRY_H

#include "imu_types.h"

void imu_side_get_axis(imu_side_t side, uint8_t *index, int8_t *sign);
imu_side_t imu_side_from_vector(imu_xyz_t *vector);

float imu_vector_norm(imu_xyz_t *input);
int imu_vector_normalize(imu_xyz_t *input, imu_xyz_t *output);
void imu_vector_cross(imu_xyz_t *a, imu_xyz_t *b, imu_xyz_t *output);
int imu_vector_project(imu_xyz_t *axis, imu_xyz_t *input, float *proj);
void imu_vector_map(imu_matrix_3x3_t *map, imu_xyz_t *input,
                    imu_xyz_t *output);

int imu_map_from_base_axes(imu_xyz_t *x_axis, imu_xyz_t *z_axis,
                           imu_matrix_3x3_t *rotation);

void imu_quaternion_to_rotation_matrix(imu_quaternion_t *q,
                                       imu_matrix_3x3_t *r);
void imu_quaternion_from_gravity(imu_xyz_t *g, imu_quaternion_t *q);
void imu_quaternion_get_rate_of_change(imu_quaternion_t *q, imu_xyz_t *w,
                                       imu_quaternion_t *dq);
void imu_quaternion_normalize(imu_quaternion_t *q);

float imu_maxf(float a, float b);
float imu_absf(float a);

#endif /* __APPS_IMU_GEOMETRY_H */
