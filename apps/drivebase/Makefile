include $(APPDIR)/Make.defs

# Tick-loop test verbs (_servo, future _drive) need room for nested
# struct frames + stdio.  Default 2048 overflows; 8192 is roomy enough
# for the development surface and shrinks back to 4096 once the daemon
# CLI surface stabilises (commit #11).

PROGNAME  = drivebase
PRIORITY  = $(CONFIG_APP_DRIVEBASE_CLI_PRIORITY)
STACKSIZE = 8192
MODULE    = $(CONFIG_APP_DRIVEBASE)

MAINSRC = drivebase_main.c

CSRCS  += drivebase_motor.c
CSRCS  += drivebase_settings.c
CSRCS  += drivebase_trajectory.c
CSRCS  += drivebase_observer.c
CSRCS  += drivebase_control.c
CSRCS  += drivebase_servo.c
CSRCS  += drivebase_drivebase.c

include $(APPDIR)/Application.mk
