/****************************************************************************
 * apps/btsensor/btnsh_main.c
 *
 * NSH child task entry for the BT-side shell mode (Issue #108).
 *
 * Spawned by btsensor_shell_enter() via task_spawn() with fd 0/1/2
 * already redirected to the FIFO pair /dev/btnsh_in / /dev/btnsh_out.
 * Cannot reuse nsh_consolemain() because the usbnsh defconfig has
 * CONFIG_CDCACM_CONSOLE=y, which causes nsh_consolemain to compile to
 * the USB-console variant (nsh_usbconsole.c) that re-opens /dev/console
 * instead of inheriting fd 0/1/2.
 *
 * isctty=false because the FIFO is not a tty: nsh_builtin will skip
 * TIOCSCTTY/TIOCNOTTY, and Ctrl-C / job control is intentionally not
 * available on the BT shell.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>

#include "nsh.h"
#include "nsh_console.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int btnsh_main(int argc, FAR char *argv[])
{
  FAR struct console_stdio_s *pstate = nsh_newconsole(false);
  int ret;

  if (pstate == NULL)
    {
      return EXIT_FAILURE;
    }

  ret = nsh_session(pstate, NSH_LOGIN_LOCAL, argc, argv);

  /* Releases console_stdio_s and exits the task; mirrors nsh_system(). */

  nsh_exit(&pstate->cn_vtbl, ret);
  return ret;                          /* unreachable */
}
