include $(APPDIR)/Make.defs

PROGNAME  = drivebase
PRIORITY  = $(CONFIG_APP_DRIVEBASE_CLI_PRIORITY)
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE    = $(CONFIG_APP_DRIVEBASE)

MAINSRC = drivebase_main.c

CSRCS  += drivebase_motor.c
CSRCS  += drivebase_settings.c
CSRCS  += drivebase_trajectory.c
CSRCS  += drivebase_observer.c
CSRCS  += drivebase_control.c

include $(APPDIR)/Application.mk
