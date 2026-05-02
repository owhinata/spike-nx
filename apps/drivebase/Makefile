include $(APPDIR)/Make.defs

# CLI is short-lived: open() + ioctl() + printf() per verb.  4 KB
# covers the production surface (start / stop / status / config /
# straight / turn / forever / get-state / jitter).  Test verbs
# (_drive / _daemon / _servo) keep their large struct frames in
# file-scope static instead of on the stack so they also fit.
#
# The 8 KB previous default starved user heap when the daemon was
# already running: NSH spawning a second 8 KB CLI task on top of the
# daemon's 8 KB main + 4 KB RT pthread stacks failed with -ENOMEM,
# and exec_builtin reported it to NSH as "command not found"
# (CONFIG_NSH_FILE_APPS / CONFIG_LIBC_EXECFUNCS both off → no
# fallback path that surfaces the real errno).

PROGNAME  = drivebase
PRIORITY  = $(CONFIG_APP_DRIVEBASE_CLI_PRIORITY)
STACKSIZE = 4096
MODULE    = $(CONFIG_APP_DRIVEBASE)

MAINSRC = drivebase_main.c

CSRCS  += drivebase_motor.c
CSRCS  += drivebase_settings.c
CSRCS  += drivebase_trajectory.c
CSRCS  += drivebase_observer.c
CSRCS  += drivebase_control.c
CSRCS  += drivebase_servo.c
CSRCS  += drivebase_drivebase.c
CSRCS  += drivebase_rt.c
CSRCS  += drivebase_chardev_handler.c
CSRCS  += drivebase_imu.c
CSRCS  += drivebase_daemon.c

include $(APPDIR)/Application.mk
