include $(APPDIR)/Make.defs

PROGNAME = imu
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE = $(CONFIG_APP_IMU)

MAINSRC = imu_main.c
CSRCS = imu_geometry.c imu_stationary.c imu_fusion.c imu_calibration.c

include $(APPDIR)/Application.mk
