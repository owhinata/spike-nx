/****************************************************************************
 * apps/sensor/sensor_main.c
 *
 * CLI for the SPIKE Prime Hub LEGO sensor uORB driver (Issue #79).
 *
 * Six device-class subcommands, each opening the matching
 * `/dev/uorb/sensor_*` topic:
 *
 *   color | ultrasonic | force | motor_m | motor_r | motor_l
 *
 * Each subcommand takes a verb:
 *
 *   <class>                          status one-liner (default)
 *   <class> info                     dump device info / mode schema
 *   <class> status                   engine + traffic counters
 *   <class> watch [ms]               poll + decode samples (default 1000)
 *   <class> select <mode>            open → CLAIM → SELECT → close (auto-RELEASE)
 *   <class> send <mode> <hex>...     open → CLAIM → SEND  → close
 *   <class> pwm <ch0> [ch1 ch2 ch3]  open → CLAIM → SET_PWM → close
 *
 * `sensor` / `sensor list` enumerates all six class topics.
 ****************************************************************************/

#include <nuttx/config.h>

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <arch/board/board_legosensor.h>
#include <arch/board/board_lump.h>

#ifdef CONFIG_APP_CAPTURE
#  include "capture.h"
#  include "capture_schema_color_reflection_run.h"
#  include "capture_schema_color_rgbi_run.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WATCH_DEFAULT_MS    1000
#define CAPTURE_DEFAULT_MS  1000
#define SELECT_POLL_MS       500

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct class_entry_s
{
  enum legosensor_class_e id;
  const char *name;             /* CLI keyword */
  const char *path;             /* /dev/uorb/sensor_* */
  uint8_t     pwm_channels;     /* expected num_channels for SET_PWM, 0 = none */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct class_entry_s g_class_table[] =
{
  { LEGOSENSOR_CLASS_COLOR,      "color",      "/dev/uorb/sensor_color",      3 },
  { LEGOSENSOR_CLASS_ULTRASONIC, "ultrasonic", "/dev/uorb/sensor_ultrasonic", 4 },
  { LEGOSENSOR_CLASS_FORCE,      "force",      "/dev/uorb/sensor_force",      0 },
  { LEGOSENSOR_CLASS_MOTOR_M,    "motor_m",    "/dev/uorb/sensor_motor_m",    1 },
  { LEGOSENSOR_CLASS_MOTOR_R,    "motor_r",    "/dev/uorb/sensor_motor_r",    1 },
  { LEGOSENSOR_CLASS_MOTOR_L,    "motor_l",    "/dev/uorb/sensor_motor_l",    1 },
};

#define CLASS_COUNT  (int)(sizeof(g_class_table) / sizeof(g_class_table[0]))

/****************************************************************************
 * Private Functions — helpers
 ****************************************************************************/

static const struct class_entry_s *lookup_class(const char *name)
{
  for (int i = 0; i < CLASS_COUNT; i++)
    {
      if (strcmp(name, g_class_table[i].name) == 0)
        {
          return &g_class_table[i];
        }
    }

  return NULL;
}

static const char *state_name(uint8_t state)
{
  static const char * const names[] =
  {
    [LUMP_ENGINE_IDLE]    = "IDLE",
    [LUMP_ENGINE_SYNCING] = "SYNCING",
    [LUMP_ENGINE_INFO]    = "INFO",
    [LUMP_ENGINE_DATA]    = "DATA",
    [LUMP_ENGINE_ERR]     = "ERR",
  };

  if (state < sizeof(names) / sizeof(names[0]) && names[state] != NULL)
    {
      return names[state];
    }

  return "?";
}

static const char *dtype_name(uint8_t dt)
{
  switch (dt)
    {
      case LUMP_DATA_INT8:  return "INT8";
      case LUMP_DATA_INT16: return "INT16";
      case LUMP_DATA_INT32: return "INT32";
      case LUMP_DATA_FLOAT: return "FLOAT";
      default:              return "?";
    }
}

static int parse_hex_byte(const char *s, uint8_t *out)
{
  unsigned long v;
  char *end;

  if (s == NULL || s[0] == '\0')
    {
      return -EINVAL;
    }

  v = strtoul(s, &end, 16);
  if (*end != '\0' || v > 0xff)
    {
      return -EINVAL;
    }

  *out = (uint8_t)v;
  return 0;
}

static int parse_signed_long(const char *s, long *out)
{
  char *end;
  long v;

  if (s == NULL || s[0] == '\0')
    {
      return -EINVAL;
    }

  v = strtol(s, &end, 0);
  if (*end != '\0')
    {
      return -EINVAL;
    }

  *out = v;
  return 0;
}

/****************************************************************************
 * Private Functions — sample decoding
 ****************************************************************************/

/* Header used by `sensor <class> watch`.  The columns are space-padded
 * so the output is column-aligned in a fixed-width terminal, with the
 * per-sample `data` payload tail-printed because its width is mode-
 * dependent (INT8 x4 vs FLOAT x8 etc.).
 */

#define WATCH_HEADER_FMT  \
    "%8s  %4s  %5s  %6s  %4s  %-5s  %4s  %s\n"
#define WATCH_ROW_FMT     \
    "%8ld  %4c  %5" PRIu32 "  %6" PRIu32 "  %4u  %-5s  %4u  "

static void print_sample_row(long time_ms, const struct lump_sample_s *s)
{
  printf(WATCH_ROW_FMT, time_ms, 'A' + s->port,
         s->generation, s->seq, s->mode_id,
         dtype_name(s->data_type), s->num_values);

  if (s->len == 0)
    {
      printf("%s\n", s->type_id == 0 ? "<disconnect>" : "<sync>");
      return;
    }

  switch (s->data_type)
    {
      case LUMP_DATA_INT8:
        for (int i = 0; i < s->num_values && i < 32; i++)
          {
            printf("%s%d", i ? " " : "", s->data.i8[i]);
          }
        break;

      case LUMP_DATA_INT16:
        for (int i = 0; i < s->num_values && i < 16; i++)
          {
            printf("%s%d", i ? " " : "", s->data.i16[i]);
          }
        break;

      case LUMP_DATA_INT32:
        for (int i = 0; i < s->num_values && i < 8; i++)
          {
            printf("%s%" PRId32, i ? " " : "", s->data.i32[i]);
          }
        break;

      case LUMP_DATA_FLOAT:
        for (int i = 0; i < s->num_values && i < 8; i++)
          {
            printf("%s%.3f", i ? " " : "", (double)s->data.f32[i]);
          }
        break;

      default:
        for (int i = 0; i < s->len && i < 32; i++)
          {
            printf("%s%02x", i ? " " : "", s->data.raw[i]);
          }
        break;
    }
  printf("\n");
}

/****************************************************************************
 * Private Functions — subcommands
 ****************************************************************************/

/* Multi-line `key : value` snapshot used by `sensor <class>` (no verb)
 * and `sensor <class> status`.  Mirrors the old long-form output that
 * existed before the Issue #106 unification.
 */

static int do_status_one(const struct class_entry_s *c)
{
  int fd = open(c->path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      printf("%-11s  <open: %s>\n", c->name, strerror(errno));
      return -errno;
    }

  struct lump_status_full_s st;
  memset(&st, 0, sizeof(st));
  int ret = ioctl(fd, LEGOSENSOR_GET_STATUS, (unsigned long)&st);

  struct legosensor_info_arg_s info_arg;
  memset(&info_arg, 0, sizeof(info_arg));
  int ret_info = ioctl(fd, LEGOSENSOR_GET_INFO, (unsigned long)&info_arg);
  close(fd);

  if (ret < 0 && errno == ENODEV)
    {
      printf("%-11s  <unbound>\n", c->name);
      return 0;
    }

  if (ret < 0)
    {
      printf("%-11s  <ioctl: %s>\n", c->name, strerror(errno));
      return -errno;
    }

  printf("class       : %s\n", c->name);
  if (ret_info == 0)
    {
      printf("bound port  : %c\n", 'A' + info_arg.port);
    }
  printf("type_id     : %u\n", st.type_id);
  printf("state       : %s\n", state_name(st.state));
  printf("current_mode: %u\n", st.current_mode);
  printf("baud        : %" PRIu32 "\n", st.baud);
  printf("rx / tx     : %" PRIu32 " / %" PRIu32 " bytes\n",
         st.rx_bytes, st.tx_bytes);
  printf("flags       : 0x%02x%s%s%s\n", st.flags,
         (st.flags & LUMP_FLAG_SYNCED)  ? " SYNCED"  : "",
         (st.flags & LUMP_FLAG_DATA_OK) ? " DATA_OK" : "",
         (st.flags & LUMP_FLAG_ERROR)   ? " ERROR"   : "");
  printf("dq_dropped  : %" PRIu32 "\n", st.dq_dropped);
  printf("stack used  : %" PRIu32 " / %" PRIu32 "\n",
         st.stk_used, st.stk_size);

  return 0;
}

/* Class-axis tabular layout matching `port lump status` (Issue #106).
 * The first column is the class name; the rest of the row mirrors the
 * port-side table 1:1 so the two views can be read side-by-side.
 */

static int do_list(void)
{
  printf("Class        Port  State    Type  Mode  Baud    RX(B)     TX(B)   "
         "DqDrop  Err   BadMsg  Backoff  StkHWM      IsrPct  IsrAvgNs\n");
  printf("-----------  ----  -------  ----  ----  ------  --------  ------  "
         "------  ----  ------  -------  ----------  ------  --------\n");

  for (int i = 0; i < CLASS_COUNT; i++)
    {
      const struct class_entry_s *c = &g_class_table[i];

      int fd = open(c->path, O_RDONLY | O_NONBLOCK);
      if (fd < 0)
        {
          printf("%-11s  <open: %d>\n", c->name, errno);
          continue;
        }

      struct lump_status_full_s s;
      memset(&s, 0, sizeof(s));
      int ret = ioctl(fd, LEGOSENSOR_GET_STATUS, (unsigned long)&s);

      struct legosensor_info_arg_s info_arg;
      memset(&info_arg, 0, sizeof(info_arg));
      int ret_info = ioctl(fd, LEGOSENSOR_GET_INFO, (unsigned long)&info_arg);
      close(fd);

      if (ret < 0 && errno == ENODEV)
        {
          printf("%-11s   --   <unbound>\n", c->name);
          continue;
        }
      if (ret < 0)
        {
          printf("%-11s  <ioctl: %d>\n", c->name, errno);
          continue;
        }

      printf("%-11s   %c    %-7s  %3u   %3u   %6lu  %8lu  %6lu  "
             "%6lu  %4lu  %6lu  %7lu  %4lu/%4lu  %3lu.%lu  %8lu\n",
             c->name,
             ret_info == 0 ? 'A' + info_arg.port : '?',
             state_name(s.state),
             s.type_id, s.current_mode,
             (unsigned long)s.baud,
             (unsigned long)s.rx_bytes, (unsigned long)s.tx_bytes,
             (unsigned long)s.dq_dropped, (unsigned long)s.err_count,
             (unsigned long)s.bad_msg_count, (unsigned long)s.backoff_step,
             (unsigned long)s.stk_used, (unsigned long)s.stk_size,
             (unsigned long)(s.isr_pct_x10 / 10u),
             (unsigned long)(s.isr_pct_x10 % 10u),
             (unsigned long)s.isr_avg_ns);
    }

  return 0;
}

/* Port-style indented `key:` info dump (Issue #106 — same layout as
 * `port lump <P> info` so the two views can be read interchangeably).
 * Header line carries the class + bound port; body and mode rows are
 * 1:1 with the port-side print code in apps/port/port_main.c.
 */

static int do_info(const struct class_entry_s *c)
{
  int fd = open(c->path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s): %s\n", c->path, strerror(errno));
      return -errno;
    }

  struct legosensor_info_arg_s arg;
  memset(&arg, 0, sizeof(arg));
  int ret = ioctl(fd, LEGOSENSOR_GET_INFO, (unsigned long)&arg);
  close(fd);

  if (ret < 0)
    {
      fprintf(stderr, "GET_INFO: %s\n", strerror(errno));
      return -errno;
    }

  const struct lump_device_info_s *info = &arg.info;

  printf("Sensor %s (bound port %c) info:\n", c->name, 'A' + arg.port);
  printf("  type_id:      %u\n", info->type_id);
  printf("  num_modes:    %u\n", info->num_modes);
  printf("  current_mode: %u\n", info->current_mode);
  printf("  flags:        0x%02x %s%s%s\n",
         info->flags,
         (info->flags & LUMP_FLAG_SYNCED)  ? "[SYNCED]"  : "",
         (info->flags & LUMP_FLAG_DATA_OK) ? "[DATA_OK]" : "",
         (info->flags & LUMP_FLAG_ERROR)   ? "[ERROR]"   : "");
  printf("  capability:   0x%02x", info->capability_flags);
  if (info->capability_flags & LUMP_CAP_MOTOR)              printf(" MOTOR");
  if (info->capability_flags & LUMP_CAP_MOTOR_POWER)        printf(" MOTOR_POWER");
  if (info->capability_flags & LUMP_CAP_MOTOR_SPEED)        printf(" MOTOR_SPEED");
  if (info->capability_flags & LUMP_CAP_MOTOR_ABS_POS)      printf(" MOTOR_ABS_POS");
  if (info->capability_flags & LUMP_CAP_MOTOR_REL_POS)      printf(" MOTOR_REL_POS");
  if (info->capability_flags & LUMP_CAP_NEEDS_SUPPLY_PIN1)  printf(" NEEDS_SUPPLY_PIN1");
  if (info->capability_flags & LUMP_CAP_NEEDS_SUPPLY_PIN2)  printf(" NEEDS_SUPPLY_PIN2");
  printf("\n");
  printf("  baud:         %" PRIu32 "\n", info->baud);
  if (info->fw_version || info->hw_version)
    {
      printf("  fw_version:   0x%08" PRIx32 "\n", info->fw_version);
      printf("  hw_version:   0x%08" PRIx32 "\n", info->hw_version);
    }

  for (uint8_t m = 0; m < info->num_modes && m < LUMP_MAX_MODES; m++)
    {
      const struct lump_mode_info_s *mi = &info->modes[m];
      printf("  mode %u  %-12s  %u x %s%s",
             m,
             mi->name[0] ? mi->name : "?",
             mi->num_values,
             dtype_name(mi->data_type),
             mi->writable ? "  writable" : "");
      if (mi->units[0])
        {
          printf("  unit=%s", mi->units);
        }
      if (mi->raw_min != mi->raw_max)
        {
          printf("  raw=%g..%g",
                 (double)mi->raw_min, (double)mi->raw_max);
        }
      if (mi->pct_min != mi->pct_max)
        {
          printf("  pct=%g..%g",
                 (double)mi->pct_min, (double)mi->pct_max);
        }
      if (mi->si_min != mi->si_max)
        {
          printf("  si=%g..%g",
                 (double)mi->si_min, (double)mi->si_max);
        }
      if (mi->mode_flags)
        {
          printf("  flags=0x%02x", mi->mode_flags);
          if (mi->mode_flags & LUMP_MODE_FLAG_MOTOR)              printf(" MOTOR");
          if (mi->mode_flags & LUMP_MODE_FLAG_MOTOR_POWER)        printf(" PWR");
          if (mi->mode_flags & LUMP_MODE_FLAG_MOTOR_SPEED)        printf(" SPD");
          if (mi->mode_flags & LUMP_MODE_FLAG_MOTOR_ABS_POS)      printf(" ABS");
          if (mi->mode_flags & LUMP_MODE_FLAG_MOTOR_REL_POS)      printf(" REL");
          if (mi->mode_flags & LUMP_MODE_FLAG_NEEDS_SUPPLY_PIN1)  printf(" SUP1");
          if (mi->mode_flags & LUMP_MODE_FLAG_NEEDS_SUPPLY_PIN2)  printf(" SUP2");
        }
      printf("\n");
    }

  return 0;
}

static int do_watch(const struct class_entry_s *c, int duration_ms)
{
  int fd = open(c->path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s): %s\n", c->path, strerror(errno));
      return -errno;
    }

  printf(WATCH_HEADER_FMT,
         "time_ms", "port", "gen", "seq", "mode", "type", "nval", "data");

  struct timespec t0;
  clock_gettime(CLOCK_MONOTONIC, &t0);

  struct pollfd pfd = { .fd = fd, .events = POLLIN };
  int  remaining = duration_ms;
  int  count     = 0;

  while (remaining > 0)
    {
      int pr = poll(&pfd, 1, remaining);
      if (pr < 0)
        {
          if (errno == EINTR)
            {
              continue;
            }
          break;
        }

      if (pr == 0)
        {
          break;        /* timeout */
        }

      struct lump_sample_s s;
      ssize_t n = read(fd, &s, sizeof(s));

      struct timespec t1;
      clock_gettime(CLOCK_MONOTONIC, &t1);
      long elapsed_ms = (t1.tv_sec - t0.tv_sec) * 1000 +
                        (t1.tv_nsec - t0.tv_nsec) / 1000000;

      if (n == sizeof(s))
        {
          print_sample_row(elapsed_ms, &s);
          count++;
        }

      remaining = duration_ms - (int)elapsed_ms;
    }

  close(fd);
  printf("(received %d samples in %d ms)\n", count, duration_ms);
  return 0;
}

/* Quiet rate-only counterpart of `do_watch`.  Drops the per-sample
 * `printf` so the USB-CDC write path does not throttle the read loop;
 * useful when the question is purely "how many samples per second is
 * the uORB stream delivering for this class?".  Sentinel samples
 * (sync/disconnect, len==0) are counted separately so they do not
 * skew the data-rate.
 */

static int do_fps(const struct class_entry_s *c, int duration_ms)
{
  int fd = open(c->path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s): %s\n", c->path, strerror(errno));
      return -errno;
    }

  struct timespec t0;
  clock_gettime(CLOCK_MONOTONIC, &t0);

  struct pollfd pfd = { .fd = fd, .events = POLLIN };
  int  remaining = duration_ms;
  int  count     = 0;
  int  sentinels = 0;
  uint8_t last_mode = 0xff;

  while (remaining > 0)
    {
      /* Drain everything currently buffered in the uORB upper-half ring
       * before going back to poll().  Each read() pops one sample; in
       * O_NONBLOCK mode it returns -EAGAIN once the ring is empty.
       * Without this drain step, a single poll() wake would only
       * reclaim one sample no matter how many had piled up between
       * loop iterations, which capped throughput around the system
       * tick rate.
       */

      for (;;)
        {
          struct lump_sample_s s;
          ssize_t n = read(fd, &s, sizeof(s));
          if (n != (ssize_t)sizeof(s))
            {
              break;    /* -EAGAIN or short read */
            }

          if (s.len == 0)
            {
              sentinels++;
            }
          else
            {
              count++;
              last_mode = s.mode_id;
            }
        }

      struct timespec t1;
      clock_gettime(CLOCK_MONOTONIC, &t1);
      long elapsed_ms = (t1.tv_sec - t0.tv_sec) * 1000 +
                        (t1.tv_nsec - t0.tv_nsec) / 1000000;
      remaining = duration_ms - (int)elapsed_ms;
      if (remaining <= 0)
        {
          break;
        }

      int pr = poll(&pfd, 1, remaining);
      if (pr < 0)
        {
          if (errno == EINTR)
            {
              continue;
            }
          break;
        }
      if (pr == 0)
        {
          break;        /* timeout */
        }
    }

  struct timespec t1;
  clock_gettime(CLOCK_MONOTONIC, &t1);
  long actual_ms = (t1.tv_sec - t0.tv_sec) * 1000 +
                   (t1.tv_nsec - t0.tv_nsec) / 1000000;

  close(fd);

  double fps = actual_ms > 0 ? (double)count * 1000.0 / (double)actual_ms : 0.0;
  if (last_mode != 0xff)
    {
      printf("%s: %d samples in %ld ms (= %.1f fps), mode=%u",
             c->name, count, actual_ms, fps, last_mode);
    }
  else
    {
      printf("%s: %d samples in %ld ms (= %.1f fps)",
             c->name, count, actual_ms, fps);
    }
  if (sentinels > 0)
    {
      printf(", %d sentinel(s)", sentinels);
    }
  printf("\n");
  return 0;
}

#ifdef CONFIG_APP_CAPTURE

/****************************************************************************
 * Private Functions — `<class> capture` (Issue #122)
 *
 * Records uORB samples into a heap buffer for the requested duration,
 * then exports the buffer through /dev/btcap.  The export call blocks
 * until the BT-side reader (btsensor MODE CAPTURE) drains the chardev,
 * by design: there is no timeout knob, the user kills the task with
 * `kill <pid>` if they no longer want the capture.
 *
 * Mode is implicit — we read the first sample to learn `mode_id`, then
 * resolve the schema from a small `(class_id, mode_id)` table.  This
 * matches the existing `select` -> `watch` workflow where the mode is
 * configured separately from the per-call verb.
 ****************************************************************************/

/* SIGINT/SIGTERM abort flag.  Handler is flag-only (signal-async-safe);
 * the capture loop polls this between samples to short-circuit a long
 * `duration_ms`.  Per-task task-side double-invocation is prevented by
 * the kernel chardev: REGISTER_SESSION returns -EBUSY when another
 * session is in flight, so an apps-process-wide `g_capture_busy` is
 * not needed (and would leak on `kill -9`).
 */

static volatile sig_atomic_t g_capture_aborting;

struct capture_schema_entry_s
{
  enum legosensor_class_e        class_id;
  uint8_t                        mode_id;
  FAR const capture_schema_t    *schema;
};

static const struct capture_schema_entry_s g_capture_schemas[] =
{
  { LEGOSENSOR_CLASS_COLOR, 1, &g_capture_schema_color_reflection_run },
  { LEGOSENSOR_CLASS_COLOR, 5, &g_capture_schema_color_rgbi_run        },
};

#define CAPTURE_SCHEMA_COUNT \
  (int)(sizeof(g_capture_schemas) / sizeof(g_capture_schemas[0]))

static FAR const capture_schema_t *
resolve_capture_schema(enum legosensor_class_e class_id, uint8_t mode_id)
{
  for (int i = 0; i < CAPTURE_SCHEMA_COUNT; i++)
    {
      if (g_capture_schemas[i].class_id == class_id &&
          g_capture_schemas[i].mode_id  == mode_id)
        {
          return g_capture_schemas[i].schema;
        }
    }

  return NULL;
}

static void capture_sigint_handler(int sig)
{
  (void)sig;
  g_capture_aborting = 1;
}

/* Translate a fresh `lump_sample_s` into a per-schema record.  The
 * schema table is kept narrow (color reflection + color RGBI) for v1
 * so a `switch` on schema magic is enough; the table grows in lock-
 * step with new schemas.  Caller passes the running `t0` so every row
 * carries a CLOCK_BOOTTIME-equivalent us timestamp.
 */

static int convert_sample(FAR const capture_schema_t *schema,
                          FAR const struct lump_sample_s *s,
                          uint64_t t0_us, uint64_t now_us,
                          FAR void *out_record)
{
  uint32_t ts_us = (uint32_t)(now_us - t0_us);

  if (schema->magic == 0x0010)         /* color_reflection_run */
    {
      struct capture_color_reflection_run_record_s rec = {0};
      rec.ts_us = ts_us;

      if (s->data_type == LUMP_DATA_INT8 && s->num_values >= 1)
        {
          int v = s->data.i8[0];
          if (v < 0)   v = 0;
          if (v > 100) v = 100;
          rec.reflection_pct = (uint8_t)v;
        }

      memcpy(out_record, &rec, sizeof(rec));
      return (int)sizeof(rec);
    }

  if (schema->magic == 0x0011)         /* color_rgbi_run */
    {
      struct capture_color_rgbi_run_record_s rec = {0};
      rec.ts_us = ts_us;

      if (s->data_type == LUMP_DATA_INT8 && s->num_values >= 4)
        {
          rec.red       = (uint8_t)(s->data.i8[0] < 0 ? 0 : s->data.i8[0]);
          rec.green     = (uint8_t)(s->data.i8[1] < 0 ? 0 : s->data.i8[1]);
          rec.blue      = (uint8_t)(s->data.i8[2] < 0 ? 0 : s->data.i8[2]);
          rec.intensity = (uint8_t)(s->data.i8[3] < 0 ? 0 : s->data.i8[3]);
        }

      memcpy(out_record, &rec, sizeof(rec));
      return (int)sizeof(rec);
    }

  return -ENOTSUP;
}

static int do_capture(const struct class_entry_s *c, int duration_ms)
{
  int rc = 0;
  int sensor_fd = -1;
  uint8_t *buf  = NULL;
  size_t bufsize = 0;
  size_t offset  = 0;
  bool sigint_installed = false;
  bool sigterm_installed = false;
  bool init_done = false;
  capture_handle_t h;
  struct sigaction old_sa;
  struct sigaction old_term_sa;

  /* 2. Open the uORB topic and read the first sample to learn the
   * current mode (`select` is a separate verb).
   */

  sensor_fd = open(c->path, O_RDONLY | O_NONBLOCK);
  if (sensor_fd < 0)
    {
      fprintf(stderr, "open(%s): %s\n", c->path, strerror(errno));
      rc = -errno;
      goto out;
    }

  struct pollfd pfd = { .fd = sensor_fd, .events = POLLIN };
  struct lump_sample_s first;

  /* Wait up to the user's duration for the first sample.  If the LUMP
   * port has not finished syncing yet we time out and bail rather than
   * blocking forever.
   */

  int pr = poll(&pfd, 1, duration_ms);
  if (pr <= 0 || (pfd.revents & POLLIN) == 0)
    {
      fprintf(stderr, "capture: no sample within %d ms\n", duration_ms);
      rc = -ETIMEDOUT;
      goto out;
    }

  ssize_t n = read(sensor_fd, &first, sizeof(first));
  if (n != sizeof(first))
    {
      fprintf(stderr, "capture: short read (%ld)\n", (long)n);
      rc = -EIO;
      goto out;
    }

  /* 3. Resolve the (class, mode) -> schema. */

  FAR const capture_schema_t *schema =
    resolve_capture_schema(c->id, first.mode_id);
  if (schema == NULL)
    {
      fprintf(stderr,
              "capture: no schema for class=%s mode=%u\n",
              c->name, first.mode_id);
      rc = -ENOENT;
      goto out;
    }

  /* 4. Heap budget.  rate_hz_hint * duration_ms / 1000 with 1.5x
   * head-room, capped by CONFIG_APP_CAPTURE_MAX_HEAP_BYTES.
   */

  uint32_t records_est = (schema->rate_hz_hint > 0)
    ? ((uint32_t)duration_ms * schema->rate_hz_hint + 999) / 1000 + 16
    : 256;

  bufsize = (size_t)records_est * schema->record_size;
  if (bufsize > CONFIG_APP_CAPTURE_MAX_HEAP_BYTES)
    {
      bufsize = CONFIG_APP_CAPTURE_MAX_HEAP_BYTES
              / schema->record_size * schema->record_size;
    }

  if (bufsize < schema->record_size)
    {
      fprintf(stderr, "capture: record_size > MAX_HEAP_BYTES\n");
      rc = -EINVAL;
      goto out;
    }

  buf = (uint8_t *)malloc(bufsize);
  if (buf == NULL)
    {
      fprintf(stderr, "capture: malloc(%zu) failed\n", bufsize);
      rc = -ENOMEM;
      goto out;
    }

  /* 5. SIGINT / SIGTERM handler (flag-only).  NuttX has no default
   * "terminate task" action for SIGTERM, so without a handler `kill
   * <pid>` (which sends SIGTERM) leaves the task stuck in usleep.  We
   * catch both and let the loop / capture_deinit observe `g_aborting`
   * to wind down cleanly.  `kill -9` (SIGKILL) is still the escape
   * hatch — the kernel release fop reclaims /dev/btcap regardless.
   */

  struct sigaction new_sa = {0};
  new_sa.sa_handler = capture_sigint_handler;
  sigemptyset(&new_sa.sa_mask);

  if (sigaction(SIGINT, &new_sa, &old_sa) == 0)
    {
      sigint_installed = true;
    }

  if (sigaction(SIGTERM, &new_sa, &old_term_sa) == 0)
    {
      sigterm_installed = true;
    }

  /* 6. Capture phase.  The first sample we already read becomes the
   * first row; subsequent rows come from the poll loop.  CLOCK_BOOTTIME
   * timestamps are relative to the first sample so the .cap viewer
   * gets a 0-based time axis.
   */

  struct timespec ts_now;
  clock_gettime(CLOCK_BOOTTIME, &ts_now);
  uint64_t t0_us = (uint64_t)ts_now.tv_sec * 1000000ull
                 + (uint64_t)ts_now.tv_nsec / 1000ull;
  uint64_t now_us = t0_us;

  int rec_bytes = convert_sample(schema, &first, t0_us, now_us,
                                 &buf[offset]);
  if (rec_bytes < 0)
    {
      rc = rec_bytes;
      goto out;
    }

  offset += (size_t)rec_bytes;

  while (!g_capture_aborting && (offset + schema->record_size) <= bufsize)
    {
      clock_gettime(CLOCK_BOOTTIME, &ts_now);
      uint64_t cur_us = (uint64_t)ts_now.tv_sec * 1000000ull
                      + (uint64_t)ts_now.tv_nsec / 1000ull;

      long elapsed_ms = (long)((cur_us - t0_us) / 1000ull);
      if (elapsed_ms >= duration_ms)
        {
          break;
        }

      int remaining_ms = duration_ms - (int)elapsed_ms;
      pr = poll(&pfd, 1, remaining_ms);
      if (pr < 0)
        {
          if (errno == EINTR) continue;
          rc = -errno;
          break;
        }

      if (pr == 0)
        {
          break;
        }

      struct lump_sample_s s;
      n = read(sensor_fd, &s, sizeof(s));
      if (n != sizeof(s))
        {
          continue;
        }

      if (s.mode_id != first.mode_id)
        {
          fprintf(stderr,
                  "capture: mode changed mid-capture (was %u, now %u)\n",
                  first.mode_id, s.mode_id);
          rc = -EILSEQ;
          break;
        }

      clock_gettime(CLOCK_BOOTTIME, &ts_now);
      now_us = (uint64_t)ts_now.tv_sec * 1000000ull
             + (uint64_t)ts_now.tv_nsec / 1000ull;

      rec_bytes = convert_sample(schema, &s, t0_us, now_us, &buf[offset]);
      if (rec_bytes < 0)
        {
          rc = rec_bytes;
          break;
        }

      offset += (size_t)rec_bytes;
    }

  if (rc < 0)
    {
      goto out;
    }

  uint32_t nrec = (uint32_t)(offset / schema->record_size);
  if (nrec == 0)
    {
      fprintf(stderr, "capture: no records collected\n");
      rc = -ENODATA;
      goto out;
    }

  printf("capture: %lu records over %d ms (schema=%s)\n",
         (unsigned long)nrec, duration_ms, schema->name);
  printf("capture: waiting for `btsensor mode capture` (or BT MODE "
         "CAPTURE) to drain /dev/btcap...\n");

  /* 7. Export.  capture_write() blocks until the reader engages — that
   * is the user's cue to invoke `btsensor mode capture` from a second
   * NSH session (or send `MODE CAPTURE` over BT).
   */

  rc = capture_init(&h, schema, nrec);
  if (rc < 0)
    {
      fprintf(stderr, "capture_init: %d\n", rc);
      goto out;
    }

  init_done = true;

  rc = capture_write(&h, buf, (size_t)nrec * schema->record_size);
  if (rc < 0)
    {
      fprintf(stderr, "capture_write: %d\n", rc);
      capture_abort(&h);
      init_done = false;
      goto out;
    }

  rc = capture_deinit(&h);
  init_done = false;
  if (rc < 0)
    {
      fprintf(stderr, "capture_deinit: %d\n", rc);
      goto out;
    }

  printf("capture: done\n");

out:
  if (init_done)
    {
      capture_abort(&h);
    }

  if (sigint_installed)
    {
      sigaction(SIGINT, &old_sa, NULL);
    }

  if (sigterm_installed)
    {
      sigaction(SIGTERM, &old_term_sa, NULL);
    }

  if (buf != NULL)
    {
      free(buf);
    }

  if (sensor_fd >= 0)
    {
      close(sensor_fd);
    }

  g_capture_aborting = 0;
  return rc;
}

#endif /* CONFIG_APP_CAPTURE */

static int do_select(const struct class_entry_s *c, uint8_t mode)
{
  int fd = open(c->path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s): %s\n", c->path, strerror(errno));
      return -errno;
    }

  int ret = ioctl(fd, LEGOSENSOR_CLAIM, 0);
  if (ret < 0)
    {
      fprintf(stderr, "CLAIM: %s\n", strerror(errno));
      close(fd);
      return -errno;
    }

  struct legosensor_select_arg_s sel = { .mode = mode };
  ret = ioctl(fd, LEGOSENSOR_SELECT, (unsigned long)&sel);
  if (ret < 0)
    {
      fprintf(stderr, "SELECT: %s\n", strerror(errno));
      close(fd);                     /* auto-RELEASE on close */
      return -errno;
    }

  /* Wait up to SELECT_POLL_MS for a sample whose mode matches.  This
   * lets the CLI report a clean "switched to mode N" line instead of
   * exiting before the device acknowledges the request.
   */

  struct pollfd pfd = { .fd = fd, .events = POLLIN };
  struct timespec t0;
  clock_gettime(CLOCK_MONOTONIC, &t0);

  bool confirmed = false;
  int  remaining = SELECT_POLL_MS;
  while (remaining > 0 && !confirmed)
    {
      int pr = poll(&pfd, 1, remaining);
      if (pr <= 0)
        {
          break;
        }

      struct lump_sample_s s;
      ssize_t n = read(fd, &s, sizeof(s));
      if (n == sizeof(s) && s.mode_id == mode && s.len > 0)
        {
          confirmed = true;
          break;
        }

      struct timespec t1;
      clock_gettime(CLOCK_MONOTONIC, &t1);
      long elapsed_ms = (t1.tv_sec - t0.tv_sec) * 1000 +
                        (t1.tv_nsec - t0.tv_nsec) / 1000000;
      remaining = SELECT_POLL_MS - (int)elapsed_ms;
    }

  close(fd);                          /* auto-RELEASE */

  if (!confirmed)
    {
      fprintf(stderr, "SELECT timeout (mode %u not confirmed in %d ms)\n",
              mode, SELECT_POLL_MS);
      return -ETIMEDOUT;
    }

  printf("%s switched to mode %u\n", c->name, mode);
  return 0;
}

static int do_send(const struct class_entry_s *c, uint8_t mode,
                   int argc, char **argv)
{
  if (argc <= 0)
    {
      fprintf(stderr, "usage: sensor %s send <m> <hex>...\n", c->name);
      return -EINVAL;
    }

  if (argc > LUMP_MAX_PAYLOAD)
    {
      fprintf(stderr, "payload exceeds %d bytes\n", LUMP_MAX_PAYLOAD);
      return -E2BIG;
    }

  struct legosensor_send_arg_s snd;
  memset(&snd, 0, sizeof(snd));
  snd.mode = mode;
  snd.len  = (uint8_t)argc;

  for (int i = 0; i < argc; i++)
    {
      int r = parse_hex_byte(argv[i], &snd.data[i]);
      if (r < 0)
        {
          fprintf(stderr, "bad hex byte: %s\n", argv[i]);
          return -EINVAL;
        }
    }

  int fd = open(c->path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s): %s\n", c->path, strerror(errno));
      return -errno;
    }

  int ret = ioctl(fd, LEGOSENSOR_CLAIM, 0);
  if (ret < 0)
    {
      fprintf(stderr, "CLAIM: %s\n", strerror(errno));
      close(fd);
      return -errno;
    }

  ret = ioctl(fd, LEGOSENSOR_SEND, (unsigned long)&snd);
  if (ret < 0)
    {
      fprintf(stderr, "SEND: %s\n", strerror(errno));
    }

  close(fd);                            /* auto-RELEASE */
  return ret < 0 ? -errno : 0;
}

/* Resolve the per-class motor coast/brake ioctl.  Each motor class owns
 * its own ioctl number in the reserved LEGOSENSOR_MOTOR_*_BASE range so
 * the kernel can verify the caller's class matches the requested action.
 * Non-motor classes fall through to -ENOTTY at the kernel side; here we
 * report a clearer error early.
 */

static int motor_actuate_cmd(const struct class_entry_s *c, bool brake)
{
  switch (c->id)
    {
      case LEGOSENSOR_CLASS_MOTOR_M:
        return brake ? LEGOSENSOR_MOTOR_M_BRAKE : LEGOSENSOR_MOTOR_M_COAST;
      case LEGOSENSOR_CLASS_MOTOR_R:
        return brake ? LEGOSENSOR_MOTOR_R_BRAKE : LEGOSENSOR_MOTOR_R_COAST;
      case LEGOSENSOR_CLASS_MOTOR_L:
        return brake ? LEGOSENSOR_MOTOR_L_BRAKE : LEGOSENSOR_MOTOR_L_COAST;
      default:
        return -1;
    }
}

static int do_motor_actuate(const struct class_entry_s *c, bool brake)
{
  int cmd = motor_actuate_cmd(c, brake);
  if (cmd < 0)
    {
      fprintf(stderr,
              "%s: %s only valid for motor_m / motor_r / motor_l\n",
              c->name, brake ? "brake" : "coast");
      return -ENOTSUP;
    }

  int fd = open(c->path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s): %s\n", c->path, strerror(errno));
      return -errno;
    }

  int ret = ioctl(fd, LEGOSENSOR_CLAIM, 0);
  if (ret < 0)
    {
      fprintf(stderr, "CLAIM: %s\n", strerror(errno));
      close(fd);
      return -errno;
    }

  ret = ioctl(fd, cmd, 0);
  if (ret < 0)
    {
      fprintf(stderr, "%s: %s\n",
              brake ? "BRAKE" : "COAST", strerror(errno));
    }

  close(fd);                            /* auto-RELEASE */
  return ret < 0 ? -errno : 0;
}

static int do_pwm(const struct class_entry_s *c, int argc, char **argv)
{
  if (c->pwm_channels == 0)
    {
      fprintf(stderr, "%s: SET_PWM not supported for this class\n", c->name);
      return -ENOTSUP;
    }

  if (argc != c->pwm_channels)
    {
      fprintf(stderr,
              "%s: pwm needs %u channels (got %d)\n",
              c->name, c->pwm_channels, argc);
      return -EINVAL;
    }

  struct legosensor_pwm_arg_s pwm;
  memset(&pwm, 0, sizeof(pwm));
  pwm.num_channels = c->pwm_channels;

  for (int i = 0; i < argc; i++)
    {
      long v;
      if (parse_signed_long(argv[i], &v) < 0 || v < INT16_MIN || v > INT16_MAX)
        {
          fprintf(stderr, "bad channel value: %s\n", argv[i]);
          return -EINVAL;
        }
      pwm.channels[i] = (int16_t)v;
    }

  int fd = open(c->path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s): %s\n", c->path, strerror(errno));
      return -errno;
    }

  int ret = ioctl(fd, LEGOSENSOR_CLAIM, 0);
  if (ret < 0)
    {
      fprintf(stderr, "CLAIM: %s\n", strerror(errno));
      close(fd);
      return -errno;
    }

  ret = ioctl(fd, LEGOSENSOR_SET_PWM, (unsigned long)&pwm);
  if (ret < 0)
    {
      fprintf(stderr, "SET_PWM: %s\n", strerror(errno));
    }

  close(fd);                            /* auto-RELEASE */
  return ret < 0 ? -errno : 0;
}

/****************************************************************************
 * Public Function: main
 ****************************************************************************/

static void usage(void)
{
  fprintf(stderr,
          "usage:\n"
          "  sensor                                 show this help\n"
          "  sensor list                            list all class topics\n"
          "  sensor <class>                         status one-liner\n"
          "  sensor <class> info                    device info / mode schema\n"
          "  sensor <class> status                  engine + traffic counters\n"
          "  sensor <class> watch [ms]              decode samples (default 1000)\n"
          "  sensor <class> fps [ms]                rate-only count, no per-sample print\n"
#ifdef CONFIG_APP_CAPTURE
          "  sensor <class> capture [ms]            capture to /dev/btcap (Issue #122);\n"
          "                                         needs `btsensor mode capture` to drain\n"
#endif
          "  sensor <class> select <mode>           SELECT mode\n"
          "  sensor <class> send <mode> <hex>...    SEND writable-mode payload\n"
          "  sensor <class> pwm <ch0> [ch1 ch2 ch3] LED brightness / motor duty\n"
          "  sensor <motor> coast                   H-bridge open (motor_m/r/l only)\n"
          "  sensor <motor> brake                   low-side short  (motor_m/r/l only)\n"
          "  <class> ::= color | ultrasonic | force | motor_m | motor_r | motor_l\n");
}

int main(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      usage();
      return 0;
    }

  if (strcmp(argv[1], "list") == 0)
    {
      return do_list() < 0 ? 1 : 0;
    }

  if (strcmp(argv[1], "help") == 0 ||
      strcmp(argv[1], "-h")   == 0 ||
      strcmp(argv[1], "--help") == 0)
    {
      usage();
      return 0;
    }

  const struct class_entry_s *c = lookup_class(argv[1]);
  if (c == NULL)
    {
      usage();
      return 1;
    }

  /* `sensor <class>` (no verb) → status one-liner. */

  if (argc == 2)
    {
      return do_status_one(c) < 0 ? 1 : 0;
    }

  const char *verb = argv[2];

  if (strcmp(verb, "info") == 0)
    {
      return do_info(c) < 0 ? 1 : 0;
    }

  if (strcmp(verb, "status") == 0)
    {
      return do_status_one(c) < 0 ? 1 : 0;
    }

  if (strcmp(verb, "watch") == 0)
    {
      int ms = WATCH_DEFAULT_MS;
      if (argc >= 4)
        {
          ms = atoi(argv[3]);
          if (ms <= 0)
            {
              ms = WATCH_DEFAULT_MS;
            }
        }
      return do_watch(c, ms) < 0 ? 1 : 0;
    }

  if (strcmp(verb, "fps") == 0)
    {
      int ms = WATCH_DEFAULT_MS;
      if (argc >= 4)
        {
          ms = atoi(argv[3]);
          if (ms <= 0)
            {
              ms = WATCH_DEFAULT_MS;
            }
        }
      return do_fps(c, ms) < 0 ? 1 : 0;
    }

#ifdef CONFIG_APP_CAPTURE
  if (strcmp(verb, "capture") == 0)
    {
      int ms = CAPTURE_DEFAULT_MS;
      if (argc >= 4)
        {
          ms = atoi(argv[3]);
          if (ms <= 0)
            {
              ms = CAPTURE_DEFAULT_MS;
            }
        }
      return do_capture(c, ms) < 0 ? 1 : 0;
    }
#endif

  if (strcmp(verb, "select") == 0)
    {
      if (argc < 4)
        {
          usage();
          return 1;
        }
      char *end;
      unsigned long m = strtoul(argv[3], &end, 0);
      if (*end != '\0' || m >= LUMP_MAX_MODES)
        {
          usage();
          return 1;
        }
      return do_select(c, (uint8_t)m) < 0 ? 1 : 0;
    }

  if (strcmp(verb, "send") == 0)
    {
      if (argc < 4)
        {
          usage();
          return 1;
        }
      char *end;
      unsigned long m = strtoul(argv[3], &end, 0);
      if (*end != '\0' || m >= LUMP_MAX_MODES)
        {
          usage();
          return 1;
        }
      return do_send(c, (uint8_t)m, argc - 4, &argv[4]) < 0 ? 1 : 0;
    }

  if (strcmp(verb, "pwm") == 0)
    {
      return do_pwm(c, argc - 3, &argv[3]) < 0 ? 1 : 0;
    }

  if (strcmp(verb, "coast") == 0)
    {
      return do_motor_actuate(c, false) < 0 ? 1 : 0;
    }

  if (strcmp(verb, "brake") == 0)
    {
      return do_motor_actuate(c, true) < 0 ? 1 : 0;
    }

  usage();
  return 1;
}
