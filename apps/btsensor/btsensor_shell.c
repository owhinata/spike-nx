/****************************************************************************
 * apps/btsensor/btsensor_shell.c
 *
 * BT-side NSH shell mode for btsensor (Issue #108).
 *
 * Architecture summary (full design in plan-typed-karp.md):
 *   - Two FIFOs: /dev/btnsh_in (peer→NSH stdin), /dev/btnsh_out
 *     (NSH stdout/stderr→peer).  Created at daemon start, removed at
 *     daemon stop.
 *   - btsensor side: writes incoming RFCOMM bytes non-blocking into
 *     /dev/btnsh_in (drop on EAGAIN to keep the BTstack thread free).
 *   - Reader pthread: blocking poll on /dev/btnsh_out + a self-pipe;
 *     accumulates NSH stdout into a small mutex-guarded coalescing
 *     buffer and posts a context callback to the BTstack thread to
 *     request can-send-now and call rfcomm_send.
 *   - NSH child: spawned via task_spawn(btnsh_main) with fd 0/1/2
 *     redirected to the FIFO ends.
 *   - Lifecycle: TELEMETRY → SHELL_STARTING (after enter, before
 *     OK\n drain) → SHELL (after the response queue empties).  All
 *     four exit triggers (NSH exit, peer drop, mode telemetry,
 *     daemon stop) and the drain-timeout safety net funnel through
 *     btsensor_shell_exit_async().
 *
 * All public functions other than init/deinit/is_active/get_mode
 * must run on the BTstack main thread.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <spawn.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>

#include <sys/stat.h>
#include <sys/wait.h>

#include "btstack_run_loop.h"
#include "classic/rfcomm.h"

#include "btsensor_shell.h"
#include "btsensor_tx.h"

/* Cid lookup is delegated to btsensor_tx (single source of truth). */

#define btsensor_shell_get_rfcomm_cid()  btsensor_tx_get_rfcomm_cid()

#ifdef CONFIG_APP_BTSENSOR_SHELL_MODE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BTNSH_DEVPATH_IN     "/dev/btnsh_in"
#define BTNSH_DEVPATH_OUT    "/dev/btnsh_out"

#ifndef CONFIG_APP_BTSENSOR_SHELL_TX_BUF
#  define CONFIG_APP_BTSENSOR_SHELL_TX_BUF      1024
#endif

#ifndef CONFIG_APP_BTSENSOR_SHELL_NSH_STACK
#  define CONFIG_APP_BTSENSOR_SHELL_NSH_STACK   6144
#endif

#ifndef CONFIG_APP_BTSENSOR_SHELL_READER_STACK
#  define CONFIG_APP_BTSENSOR_SHELL_READER_STACK 2048
#endif

/* btnsh child priority — kept slightly below the daemon so heavy NSH
 * activity (eg. 'dmesg', 'ls /dev') cannot starve BTstack timers.
 */

#define BTNSH_CHILD_PRIORITY     90

#define BTNSH_DRAIN_TIMEOUT_MS   500
#define BTNSH_SIGKILL_TIMEOUT_MS 500

/* btsensor_cmd line buffer reset hook — implemented in btsensor_cmd.c
 * (Issue #108 Q5/Q8).  Forward-declared here to keep the cmd_feed
 * header focused on the public ASCII protocol.
 */

void btsensor_cmd_reset_rx_buffer(void);

/* btnsh child entry point lives in btnsh_main.c (Issue #108). */

int btnsh_main(int argc, FAR char *argv[]);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct shell_state_s
{
  enum btsensor_mode_e mode;        /* shared between btstack thread + spp.c via accessor */
  uint32_t             generation;  /* bumped on every enter/exit; for stale callbacks */

  pid_t                nsh_pid;     /* < 0 when no child */
  int                  fd_in;       /* btsensor → NSH stdin (O_NONBLOCK) */
  int                  fd_out;      /* NSH stdout → btsensor (blocking) */
  int                  ctrl_pipe[2];

  pthread_t            reader_tid;
  bool                 reader_running;

  /* TX coalescing buffer for NSH stdout bytes en route to RFCOMM. */

  pthread_mutex_t      tx_lock;
  uint8_t              tx_buf[CONFIG_APP_BTSENSOR_SHELL_TX_BUF];
  size_t               tx_len;
  bool                 tx_request_pending;

  btstack_context_callback_registration_t reg_pump;
  btstack_context_callback_registration_t reg_exit;
};

/* Stale-callback guard payload.  Each context registration captures the
 * generation at the time of post; the BTstack-thread side compares
 * against the current generation and bails out if they differ.
 */

struct shell_callback_ctx_s
{
  uint32_t generation;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct shell_state_s  g_shell;
static bool                  g_shell_available;
static struct shell_callback_ctx_s g_pump_ctx;
static struct shell_callback_ctx_s g_exit_ctx;
static enum btsensor_shell_reason_e g_pending_exit_reason;

/* Forward declarations. */

static void *reader_thread(void *arg);
static void  shell_pump_action(void *ctx);
static void  shell_exit_action(void *ctx);
static void  cleanup_active_shell(enum btsensor_shell_reason_e reason);
static void  do_kill_and_join(void);
static int   wait_with_timeout(pid_t pid, uint32_t timeout_ms);

/****************************************************************************
 * Private Functions — utilities
 ****************************************************************************/

static void shell_state_reset_locked(void)
{
  /* g_shell.mode and g_shell.generation are managed by the caller. */

  g_shell.nsh_pid        = -1;
  g_shell.fd_in          = -1;
  g_shell.fd_out         = -1;
  g_shell.ctrl_pipe[0]   = -1;
  g_shell.ctrl_pipe[1]   = -1;
  g_shell.reader_running = false;
  g_shell.tx_len         = 0;
  g_shell.tx_request_pending = false;
}

static void close_fd(int *fd)
{
  if (*fd >= 0)
    {
      close(*fd);
      *fd = -1;
    }
}

/* Keep writing until all bytes are sent, EAGAIN, or error.  Used by the
 * shutdown self-pipe wake-up where 1 byte is enough but EAGAIN is
 * ignored (the reader drains regardless).
 */

static void write_self_pipe_wakeup(void)
{
  if (g_shell.ctrl_pipe[1] >= 0)
    {
      const uint8_t b = 0;
      ssize_t n = write(g_shell.ctrl_pipe[1], &b, 1);
      (void)n;
    }
}

static int wait_with_timeout(pid_t pid, uint32_t timeout_ms)
{
  uint32_t elapsed_ms = 0;
  while (elapsed_ms < timeout_ms)
    {
      int status;
      pid_t r = waitpid(pid, &status, WNOHANG);
      if (r == pid)
        {
          return 0;
        }

      if (r < 0)
        {
          if (errno == ECHILD)
            {
              /* Already reaped (e.g. SIGCHLD elsewhere). */

              return 0;
            }

          return -errno;
        }

      usleep(10000);                   /* 10 ms */
      elapsed_ms += 10;
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Private Functions — reader pthread
 ****************************************************************************/

static void *reader_thread(void *arg)
{
  (void)arg;

  /* Local working copies — values needed for the lifetime of this
   * thread are captured under g_shell.tx_lock at startup so a
   * concurrent exit_async() that closes our fds cannot race.
   */

  int fd_out      = g_shell.fd_out;
  int ctrl_rx     = g_shell.ctrl_pipe[0];
  uint32_t my_gen = g_shell.generation;

  for (;;)
    {
      struct pollfd pfds[2];
      pfds[0].fd      = fd_out;
      pfds[0].events  = POLLIN;
      pfds[0].revents = 0;
      pfds[1].fd      = ctrl_rx;
      pfds[1].events  = POLLIN;
      pfds[1].revents = 0;

      int pret = poll(pfds, 2, -1);
      if (pret < 0)
        {
          if (errno == EINTR)
            {
              continue;
            }

          syslog(LOG_WARNING, "btnsh: reader poll errno=%d\n", errno);
          break;
        }

      if (pfds[1].revents & (POLLIN | POLLHUP | POLLERR))
        {
          /* Shutdown signal — drain and break. */

          uint8_t scratch[16];
          while (read(ctrl_rx, scratch, sizeof(scratch)) > 0)
            {
              /* discard */
            }

          break;
        }

      if (pfds[0].revents & POLLIN)
        {
          uint8_t local[256];
          ssize_t n = read(fd_out, local, sizeof(local));
          if (n == 0)
            {
              /* NSH child closed its stdout end — graceful EOF. */

              break;
            }

          if (n < 0)
            {
              if (errno == EINTR || errno == EAGAIN)
                {
                  continue;
                }

              syslog(LOG_WARNING, "btnsh: reader read errno=%d\n", errno);
              break;
            }

          /* Append into the coalescing buffer, dropping overflow.  Then
           * post the BTstack-thread pump action only if a request is
           * not already in flight — the lock is released before the
           * post call (Codex 1st review #7).
           */

          bool need_post = false;

          pthread_mutex_lock(&g_shell.tx_lock);

          size_t remaining = sizeof(g_shell.tx_buf) - g_shell.tx_len;
          size_t copy = (size_t)n;
          if (copy > remaining)
            {
              copy = remaining;
              syslog(LOG_WARNING,
                     "btnsh: reader tx_buf overflow, dropped %zu bytes\n",
                     (size_t)n - copy);
            }

          memcpy(g_shell.tx_buf + g_shell.tx_len, local, copy);
          g_shell.tx_len += copy;

          if (g_shell.tx_len > 0 && !g_shell.tx_request_pending)
            {
              g_shell.tx_request_pending = true;
              need_post = true;
            }

          pthread_mutex_unlock(&g_shell.tx_lock);

          if (need_post)
            {
              g_pump_ctx.generation     = my_gen;
              g_shell.reg_pump.callback = shell_pump_action;
              g_shell.reg_pump.context  = &g_pump_ctx;
              btstack_run_loop_execute_on_main_thread(&g_shell.reg_pump);
            }

          continue;
        }

      if (pfds[0].revents & (POLLHUP | POLLERR))
        {
          break;
        }
    }

  /* Loop exit: notify the BTstack thread the shell should tear down.
   * The actual close/kill/cleanup runs on the BTstack thread to keep
   * btstack_* and rfcomm_* calls single-threaded.
   */

  g_exit_ctx.generation     = my_gen;
  g_shell.reg_exit.callback = shell_exit_action;
  g_shell.reg_exit.context  = &g_exit_ctx;
  g_pending_exit_reason     = BTSENSOR_SHELL_REASON_NSH_EXIT;
  btstack_run_loop_execute_on_main_thread(&g_shell.reg_exit);

  return NULL;
}

/****************************************************************************
 * Private Functions — BTstack thread callbacks
 ****************************************************************************/

static void shell_pump_action(void *ctx)
{
  struct shell_callback_ctx_s *c = ctx;
  if (c->generation != g_shell.generation)
    {
      return;                          /* stale */
    }

  if (g_shell.mode != BTSENSOR_MODE_SHELL_STARTING &&
      g_shell.mode != BTSENSOR_MODE_SHELL)
    {
      return;
    }

  uint16_t cid = btsensor_shell_get_rfcomm_cid();
  if (cid == 0)
    {
      return;                          /* no peer */
    }

  /* Independent CAN_SEND_NOW request — not the TX arbiter's path.  The
   * spp packet handler routes RFCOMM_EVENT_CAN_SEND_NOW to our
   * btsensor_shell_on_can_send_now() while shell-active so the two
   * channels stay separate.
   */

  rfcomm_request_can_send_now_event(cid);
}

static void shell_exit_action(void *ctx)
{
  struct shell_callback_ctx_s *c = ctx;
  if (c->generation != g_shell.generation)
    {
      return;
    }

  if (g_shell.mode == BTSENSOR_MODE_TELEMETRY)
    {
      return;
    }

  cleanup_active_shell(g_pending_exit_reason);
}

/****************************************************************************
 * Private Functions — cleanup
 ****************************************************************************/

static void do_kill_and_join(void)
{
  /* Step 1: poke the child out of its blocked pipecommon_read by
   * writing "exit\n" to its stdin.  Three things happen in NuttX's
   * pipe driver:
   *   - the pipe buffer accepts the bytes (5 < CONFIG_DEV_FIFO_SIZE)
   *   - nxsem_post(d_rdsem) wakes the child's nxsem_wait()
   *   - the child's readline_fd() returns "exit", nsh_session()
   *     dispatches it to cmd_exit() which terminates the task
   *
   * Empirically this is the only reliable way to take the btnsh
   * child down on this NuttX configuration: kill(SIGKILL) does NOT
   * unblock a task waiting on a pipe semaphore, and merely closing
   * the writer side does not post d_rdsem either (the close path
   * decrements d_nwriters but does not wake existing waiters).
   * Writing the literal "exit\n" sidesteps both kernel limitations.
   */

  if (g_shell.fd_in >= 0)
    {
      const char *exit_cmd = "exit\n";
      ssize_t n = write(g_shell.fd_in, exit_cmd, 5);
      (void)n;                       /* best effort — child may already
                                      * have died from a different path */
    }

  close_fd(&g_shell.fd_in);

  /* Step 2: wake the reader pthread out of its poll(). */

  write_self_pipe_wakeup();

  if (g_shell.reader_running)
    {
      pthread_join(g_shell.reader_tid, NULL);
      g_shell.reader_running = false;
    }

  /* Step 3: now that the reader is gone, close parent's read end.
   * If the child is mid-write to its stdout (fd 1) the disappearance
   * of the only reader will make those writes fail (EPIPE) and the
   * task will exit shortly.
   */

  close_fd(&g_shell.fd_out);

  /* Step 4: wait for the child to exit on its own (stdin EOF or
   * stdout EPIPE took it down).  Fall back to SIGKILL only if the
   * graceful close did not work within the timeout — and even then,
   * tolerate the case where SIGKILL does not actually reap the task
   * because of a NuttX semaphore-wait quirk.
   */

  if (g_shell.nsh_pid > 0)
    {
      int rc = wait_with_timeout(g_shell.nsh_pid, BTNSH_SIGKILL_TIMEOUT_MS);
      if (rc == -ETIMEDOUT)
        {
          syslog(LOG_WARNING,
                 "btnsh: graceful exit timeout, sending SIGKILL pid=%d\n",
                 (int)g_shell.nsh_pid);
          kill(g_shell.nsh_pid, SIGKILL);
          rc = wait_with_timeout(g_shell.nsh_pid,
                                 BTNSH_SIGKILL_TIMEOUT_MS);
          if (rc == -ETIMEDOUT)
            {
              /* Last resort: child still alive.  Do NOT block on
               * `waitpid(..., 0)` — if SIGKILL did not reap the task,
               * blocking waitpid will hang forever and time out the
               * caller.  Mark the pid as orphaned and let the next
               * shell entry start fresh; the kernel will reclaim it
               * eventually.  This path also escalates to syslog ERR
               * so the operator notices.
               */

              syslog(LOG_ERR,
                     "btnsh: SIGKILL did not reap pid=%d, leaking task\n",
                     (int)g_shell.nsh_pid);
            }
        }

      g_shell.nsh_pid = -1;
    }
}

static void cleanup_active_shell(enum btsensor_shell_reason_e reason)
{
  /* Cancel any armed post-drain hook so it does not fire after we
   * tear down the shell.
   */

  btsensor_tx_clear_post_drain_callback();

  do_kill_and_join();

  /* Close all four fds.  parent's child_in / child_out copies were
   * already closed at the end of enter; we only own fd_in / fd_out
   * and ctrl_pipe[0,1] here.
   */

  close_fd(&g_shell.fd_in);
  close_fd(&g_shell.fd_out);
  close_fd(&g_shell.ctrl_pipe[0]);
  close_fd(&g_shell.ctrl_pipe[1]);

  /* Drop any pending coalesced NSH output. */

  pthread_mutex_lock(&g_shell.tx_lock);
  g_shell.tx_len = 0;
  g_shell.tx_request_pending = false;
  pthread_mutex_unlock(&g_shell.tx_lock);

  /* Reset the cmd parser's line buffer so any partial bytes from
   * before MODE SHELL or during the transition do not bleed into the
   * next telemetry-mode command (Codex 3rd review Q5/Q8).
   */

  btsensor_cmd_reset_rx_buffer();

  /* Bump the generation so any in-flight reg_pump / reg_exit posts
   * become stale and are dropped on arrival.
   */

  g_shell.generation += 1;

  /* Mode flip first, then optionally enqueue READY\n. */

  g_shell.mode = BTSENSOR_MODE_TELEMETRY;

  if (reason != BTSENSOR_SHELL_REASON_PEER_CLOSED &&
      reason != BTSENSOR_SHELL_REASON_TX_DRAIN_TIMEOUT)
    {
      uint16_t cid = btsensor_shell_get_rfcomm_cid();
      if (cid != 0)
        {
          (void)btsensor_tx_enqueue_response("READY\n");
        }
    }

  syslog(LOG_INFO, "btnsh: exited (reason=%d)\n", (int)reason);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int btsensor_shell_init(void)
{
  /* Initialise mutex once even when later steps fail. */

  static bool mutex_initialised;
  if (!mutex_initialised)
    {
      pthread_mutex_init(&g_shell.tx_lock, NULL);
      mutex_initialised = true;
    }

  g_shell.mode = BTSENSOR_MODE_TELEMETRY;
  g_shell.generation = 0;
  shell_state_reset_locked();

  if (mkfifo(BTNSH_DEVPATH_IN, 0666) < 0 && errno != EEXIST)
    {
      syslog(LOG_WARNING, "btnsh: mkfifo %s errno=%d\n",
             BTNSH_DEVPATH_IN, errno);
      g_shell_available = false;
      return -errno;
    }

  if (mkfifo(BTNSH_DEVPATH_OUT, 0666) < 0 && errno != EEXIST)
    {
      syslog(LOG_WARNING, "btnsh: mkfifo %s errno=%d\n",
             BTNSH_DEVPATH_OUT, errno);
      g_shell_available = false;
      return -errno;
    }

  g_shell_available = true;
  return 0;
}

void btsensor_shell_deinit(void)
{
  if (g_shell.mode != BTSENSOR_MODE_TELEMETRY)
    {
      cleanup_active_shell(BTSENSOR_SHELL_REASON_DAEMON_STOP);
    }

  /* Best-effort unlink — leave alone if removal fails. */

  unlink(BTNSH_DEVPATH_IN);
  unlink(BTNSH_DEVPATH_OUT);
  g_shell_available = false;
}

bool btsensor_shell_is_active(void)
{
  return g_shell.mode != BTSENSOR_MODE_TELEMETRY;
}

enum btsensor_mode_e btsensor_shell_get_mode(void)
{
  return g_shell.mode;
}

int btsensor_shell_enter(void)
{
  if (!g_shell_available)
    {
      return -ENODEV;
    }

  if (g_shell.mode != BTSENSOR_MODE_TELEMETRY)
    {
      return -EALREADY;
    }

  int rc = 0;
  pid_t spawned = -1;
  int child_in  = -1;
  int child_out = -1;
  posix_spawn_file_actions_t actions;
  posix_spawnattr_t attr;
  bool actions_init = false;
  bool attr_init    = false;
  pthread_t reader  = (pthread_t)0;
  bool reader_started = false;

  /* 1. Anonymous self-pipe for shutdown wake-up.  Both ends are flipped
   * to O_NONBLOCK so the reader's drain loop after waking up does not
   * block on a re-read of an already-empty pipe (the first read in the
   * drain loop returns the wakeup byte; the next would block forever
   * if left as default blocking — that was the cause of the original
   * `btsensor mode telemetry` 3 s timeout).
   */

  if (pipe(g_shell.ctrl_pipe) < 0)
    {
      rc = -errno;
      syslog(LOG_WARNING, "btnsh: pipe(ctrl) errno=%d\n", errno);
      goto fail;
    }

  if (fcntl(g_shell.ctrl_pipe[0], F_SETFL, O_NONBLOCK) < 0 ||
      fcntl(g_shell.ctrl_pipe[1], F_SETFL, O_NONBLOCK) < 0)
    {
      rc = -errno;
      syslog(LOG_WARNING, "btnsh: ctrl_pipe NONBLOCK errno=%d\n", errno);
      goto fail;
    }

  /* 2. btsensor-side FIFOs.  Open with O_NONBLOCK so the open() call
   * does not block waiting for the peer end.  fd_in stays NONBLOCK
   * for the lifetime of the shell (Codex 1st review #2); fd_out is
   * flipped to blocking once the reader pthread is started.
   */

  g_shell.fd_in = open(BTNSH_DEVPATH_IN, O_WRONLY | O_NONBLOCK);
  if (g_shell.fd_in < 0)
    {
      rc = -errno;
      syslog(LOG_WARNING, "btnsh: open %s errno=%d\n",
             BTNSH_DEVPATH_IN, errno);
      goto fail;
    }

  g_shell.fd_out = open(BTNSH_DEVPATH_OUT, O_RDONLY | O_NONBLOCK);
  if (g_shell.fd_out < 0)
    {
      rc = -errno;
      syslog(LOG_WARNING, "btnsh: open %s errno=%d\n",
             BTNSH_DEVPATH_OUT, errno);
      goto fail;
    }

  /* 3. Child-side FIFO ends.  Open NONBLOCK so the open() returns
   * regardless of the other end's state, then clear NONBLOCK so
   * the inherited fd is blocking once the child runs.
   */

  child_in = open(BTNSH_DEVPATH_IN, O_RDONLY | O_NONBLOCK);
  if (child_in < 0)
    {
      rc = -errno;
      goto fail;
    }

  if (fcntl(child_in, F_SETFL, 0) < 0)
    {
      rc = -errno;
      goto fail;
    }

  child_out = open(BTNSH_DEVPATH_OUT, O_WRONLY | O_NONBLOCK);
  if (child_out < 0)
    {
      rc = -errno;
      goto fail;
    }

  if (fcntl(child_out, F_SETFL, 0) < 0)
    {
      rc = -errno;
      goto fail;
    }

  /* 4. posix_spawn setup. */

  if (posix_spawn_file_actions_init(&actions) != 0)
    {
      rc = -ENOMEM;
      goto fail;
    }

  actions_init = true;

  if (posix_spawnattr_init(&attr) != 0)
    {
      rc = -ENOMEM;
      goto fail;
    }

  attr_init = true;

  if (posix_spawn_file_actions_adddup2(&actions, child_in, 0)  != 0 ||
      posix_spawn_file_actions_adddup2(&actions, child_out, 1) != 0 ||
      posix_spawn_file_actions_adddup2(&actions, child_out, 2) != 0 ||
      posix_spawn_file_actions_addclose(&actions, child_in)    != 0 ||
      posix_spawn_file_actions_addclose(&actions, child_out)   != 0)
    {
      rc = -EIO;
      goto fail;
    }

  posix_spawnattr_setstacksize(&attr, CONFIG_APP_BTSENSOR_SHELL_NSH_STACK);

  struct sched_param sched;
  sched.sched_priority = BTNSH_CHILD_PRIORITY;
  posix_spawnattr_setschedparam(&attr, &sched);
  posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSCHEDPARAM);

  /* 5. Launch the child.  task_spawn() returns the child's pid
   * directly (NuttX non-standard ABI; not the POSIX pid_t* form).
   */

  spawned = task_spawn("btnsh", btnsh_main, &actions, &attr, NULL, NULL);
  if (spawned < 0)
    {
      rc = -errno;
      syslog(LOG_WARNING, "btnsh: task_spawn errno=%d\n", errno);
      spawned = -1;
      goto fail;
    }

  g_shell.nsh_pid = spawned;

  /* 6. Parent no longer needs the child's fd copies. */

  close(child_in);
  child_in = -1;
  close(child_out);
  child_out = -1;

  posix_spawn_file_actions_destroy(&actions);
  actions_init = false;
  posix_spawnattr_destroy(&attr);
  attr_init = false;

  /* 7. fd_out switches to blocking — the reader pthread relies on
   * blocking read() with poll() doing the wait.
   */

  if (fcntl(g_shell.fd_out, F_SETFL, 0) < 0)
    {
      rc = -errno;
      goto fail;
    }

  /* 8. Reader pthread.  Bump the generation BEFORE the pthread starts
   * so it captures the new value into its local my_gen.
   */

  g_shell.generation += 1;

  pthread_attr_t pthr_attr;
  pthread_attr_init(&pthr_attr);
  pthread_attr_setstacksize(&pthr_attr,
                            CONFIG_APP_BTSENSOR_SHELL_READER_STACK);

  if (pthread_create(&reader, &pthr_attr, reader_thread, NULL) != 0)
    {
      rc = -errno;
      pthread_attr_destroy(&pthr_attr);
      goto fail;
    }

  pthread_attr_destroy(&pthr_attr);
  g_shell.reader_tid     = reader;
  g_shell.reader_running = true;
  reader_started         = true;

  /* 9. Mode flip into STARTING — RFCOMM RX is now diverted away from
   * the cmd parser by the spp.c handler.
   */

  g_shell.mode = BTSENSOR_MODE_SHELL_STARTING;

  syslog(LOG_INFO, "btnsh: shell mode entered, pid=%d gen=%u\n",
         (int)g_shell.nsh_pid, (unsigned)g_shell.generation);
  return 0;

fail:
  /* Roll back any partial state.  Any of these resources may already
   * have been released; close_fd / close are idempotent w.r.t. -1.
   */

  if (reader_started)
    {
      write_self_pipe_wakeup();
      pthread_join(reader, NULL);
      g_shell.reader_running = false;
    }

  if (spawned > 0)
    {
      kill(spawned, SIGKILL);
      int status;
      waitpid(spawned, &status, 0);
      g_shell.nsh_pid = -1;
    }

  if (actions_init)
    {
      posix_spawn_file_actions_destroy(&actions);
    }

  if (attr_init)
    {
      posix_spawnattr_destroy(&attr);
    }

  if (child_in  >= 0) close(child_in);
  if (child_out >= 0) close(child_out);

  close_fd(&g_shell.fd_in);
  close_fd(&g_shell.fd_out);
  close_fd(&g_shell.ctrl_pipe[0]);
  close_fd(&g_shell.ctrl_pipe[1]);

  /* Mode is unchanged (TELEMETRY) on failure. */

  return rc;
}

void btsensor_shell_exit_async(enum btsensor_shell_reason_e reason)
{
  if (g_shell.mode == BTSENSOR_MODE_TELEMETRY)
    {
      return;
    }

  cleanup_active_shell(reason);
}

void btsensor_shell_on_rfcomm_data(const uint8_t *data, uint16_t len)
{
  if (g_shell.mode != BTSENSOR_MODE_SHELL || g_shell.fd_in < 0 || len == 0)
    {
      return;
    }

  ssize_t n = write(g_shell.fd_in, data, len);
  if (n < 0)
    {
      if (errno == EAGAIN)
        {
          syslog(LOG_WARNING, "btnsh: stdin overflow, dropped %u bytes\n",
                 (unsigned)len);
          return;
        }

      syslog(LOG_WARNING, "btnsh: stdin write errno=%d\n", errno);
      return;
    }

  if ((uint16_t)n < len)
    {
      syslog(LOG_WARNING, "btnsh: stdin partial write %zd/%u\n",
             n, (unsigned)len);
    }
}

void btsensor_shell_on_can_send_now(void)
{
  if (g_shell.mode != BTSENSOR_MODE_SHELL_STARTING &&
      g_shell.mode != BTSENSOR_MODE_SHELL)
    {
      return;
    }

  uint16_t cid = btsensor_shell_get_rfcomm_cid();
  if (cid == 0)
    {
      return;
    }

  /* Snapshot the coalescing buffer under lock, then send outside the
   * lock so reader pthread is not blocked while RFCOMM is busy.
   */

  uint8_t local[CONFIG_APP_BTSENSOR_SHELL_TX_BUF];
  size_t  send_len = 0;
  bool    request_more;

  pthread_mutex_lock(&g_shell.tx_lock);
  if (g_shell.tx_len > 0)
    {
      send_len = g_shell.tx_len;
      memcpy(local, g_shell.tx_buf, send_len);
      g_shell.tx_len = 0;
    }

  g_shell.tx_request_pending = false;
  request_more = false;                /* set below if reader filled
                                        * concurrently — actually no: we
                                        * just cleared it above. */
  pthread_mutex_unlock(&g_shell.tx_lock);

  if (send_len > 0)
    {
      uint8_t err = rfcomm_send(cid, local, (uint16_t)send_len);
      if (err != 0)
        {
          /* rfcomm_send rejected — typically because credit ran out
           * synchronously.  Return data to the front of the buffer and
           * re-arm the request.
           */

          pthread_mutex_lock(&g_shell.tx_lock);
          /* Push back into the buffer at offset 0; if reader has
           * already appended to the buffer, prefer the older bytes
           * (stdout ordering).
           */

          if (g_shell.tx_len + send_len <= sizeof(g_shell.tx_buf))
            {
              memmove(g_shell.tx_buf + send_len,
                      g_shell.tx_buf, g_shell.tx_len);
              memcpy(g_shell.tx_buf, local, send_len);
              g_shell.tx_len += send_len;
            }
          else
            {
              syslog(LOG_WARNING,
                     "btnsh: rfcomm_send rc=%u, %zu bytes dropped\n",
                     (unsigned)err, send_len);
            }

          if (g_shell.tx_len > 0 && !g_shell.tx_request_pending)
            {
              g_shell.tx_request_pending = true;
              request_more = true;
            }

          pthread_mutex_unlock(&g_shell.tx_lock);
        }
    }

  /* Reader may have appended bytes between our snapshot and now.
   * Re-arm if tx_len is non-zero.
   */

  pthread_mutex_lock(&g_shell.tx_lock);
  if (g_shell.tx_len > 0 && !g_shell.tx_request_pending)
    {
      g_shell.tx_request_pending = true;
      request_more = true;
    }

  pthread_mutex_unlock(&g_shell.tx_lock);

  if (request_more)
    {
      rfcomm_request_can_send_now_event(cid);
    }
}

void btsensor_shell_on_rfcomm_closed(void)
{
  if (g_shell.mode == BTSENSOR_MODE_TELEMETRY)
    {
      return;
    }

  cleanup_active_shell(BTSENSOR_SHELL_REASON_PEER_CLOSED);
}

void btsensor_shell_transition_to_active(void *ctx)
{
  (void)ctx;

  if (g_shell.mode != BTSENSOR_MODE_SHELL_STARTING)
    {
      return;
    }

  g_shell.mode = BTSENSOR_MODE_SHELL;
  syslog(LOG_INFO, "btnsh: transition starting → active\n");
}

void btsensor_shell_drain_timeout(void *ctx)
{
  (void)ctx;

  if (g_shell.mode != BTSENSOR_MODE_SHELL_STARTING)
    {
      return;
    }

  syslog(LOG_WARNING, "btnsh: post-drain timeout, tearing down\n");
  cleanup_active_shell(BTSENSOR_SHELL_REASON_TX_DRAIN_TIMEOUT);
}

#endif /* CONFIG_APP_BTSENSOR_SHELL_MODE */
