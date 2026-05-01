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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WATCH_DEFAULT_MS    1000
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
      case LUMP_DATA_INT8:  return "I8";
      case LUMP_DATA_INT16: return "I16";
      case LUMP_DATA_INT32: return "I32";
      case LUMP_DATA_FLOAT: return "F32";
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

static void print_sample(const struct lump_sample_s *s)
{
  printf("[port=%c] gen=%" PRIu32 " seq=%" PRIu32 " mode=%u (%s x%u) ",
         'A' + s->port, s->generation, s->seq, s->mode_id,
         dtype_name(s->data_type), s->num_values);

  if (s->len == 0)
    {
      if (s->type_id == 0)
        {
          printf("| <disconnect>\n");
        }
      else
        {
          printf("| <sync sentinel>\n");
        }
      return;
    }

  printf("|");
  switch (s->data_type)
    {
      case LUMP_DATA_INT8:
        for (int i = 0; i < s->num_values && i < 32; i++)
          {
            printf(" %d", s->data.i8[i]);
          }
        break;

      case LUMP_DATA_INT16:
        for (int i = 0; i < s->num_values && i < 16; i++)
          {
            printf(" %d", s->data.i16[i]);
          }
        break;

      case LUMP_DATA_INT32:
        for (int i = 0; i < s->num_values && i < 8; i++)
          {
            printf(" %" PRId32, s->data.i32[i]);
          }
        break;

      case LUMP_DATA_FLOAT:
        for (int i = 0; i < s->num_values && i < 8; i++)
          {
            printf(" %.3f", (double)s->data.f32[i]);
          }
        break;

      default:
        for (int i = 0; i < s->len && i < 32; i++)
          {
            printf(" %02x", s->data.raw[i]);
          }
        break;
    }
  printf("\n");
}

/****************************************************************************
 * Private Functions — subcommands
 ****************************************************************************/

static int do_status_one(const struct class_entry_s *c, bool short_form)
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

  if (short_form)
    {
      printf("%-11s  port=%c type=%-3u state=%-7s mode=%u rx=%" PRIu32
             " tx=%" PRIu32 "\n",
             c->name,
             ret_info == 0 ? 'A' + info_arg.port : '?',
             st.type_id, state_name(st.state),
             st.current_mode, st.rx_bytes, st.tx_bytes);
    }
  else
    {
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
    }

  return 0;
}

static int do_list(void)
{
  for (int i = 0; i < CLASS_COUNT; i++)
    {
      do_status_one(&g_class_table[i], true);
    }

  return 0;
}

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

  printf("class       : %s\n", c->name);
  printf("bound port  : %c\n", 'A' + arg.port);
  printf("type_id     : %u\n", info->type_id);
  printf("num_modes   : %u\n", info->num_modes);
  printf("current_mode: %u\n", info->current_mode);
  printf("baud        : %" PRIu32 "\n", info->baud);
  printf("fw / hw     : 0x%08" PRIx32 " / 0x%08" PRIx32 "\n",
         info->fw_version, info->hw_version);
  printf("flags       : 0x%02x%s%s%s\n", info->flags,
         (info->flags & LUMP_FLAG_SYNCED)  ? " SYNCED"  : "",
         (info->flags & LUMP_FLAG_DATA_OK) ? " DATA_OK" : "",
         (info->flags & LUMP_FLAG_ERROR)   ? " ERROR"   : "");
  printf("capability  : 0x%02x", info->capability_flags);
  if (info->capability_flags & LUMP_CAP_MOTOR)              printf(" MOTOR");
  if (info->capability_flags & LUMP_CAP_MOTOR_POWER)        printf(" MOTOR_POWER");
  if (info->capability_flags & LUMP_CAP_MOTOR_SPEED)        printf(" MOTOR_SPEED");
  if (info->capability_flags & LUMP_CAP_MOTOR_ABS_POS)      printf(" MOTOR_ABS_POS");
  if (info->capability_flags & LUMP_CAP_MOTOR_REL_POS)      printf(" MOTOR_REL_POS");
  if (info->capability_flags & LUMP_CAP_NEEDS_SUPPLY_PIN1)  printf(" NEEDS_SUPPLY_PIN1");
  if (info->capability_flags & LUMP_CAP_NEEDS_SUPPLY_PIN2)  printf(" NEEDS_SUPPLY_PIN2");
  printf("\n");

  for (int m = 0; m < info->num_modes && m < LUMP_MAX_MODES; m++)
    {
      const struct lump_mode_info_s *mi = &info->modes[m];
      printf("mode[%d] %-12s  %ux%s  raw[%g..%g] pct[%g..%g] si[%g..%g] %s%s",
             m, mi->name, mi->num_values, dtype_name(mi->data_type),
             (double)mi->raw_min, (double)mi->raw_max,
             (double)mi->pct_min, (double)mi->pct_max,
             (double)mi->si_min,  (double)mi->si_max,
             mi->units, mi->writable ? " writable" : "");
      if (mi->mode_flags)
        {
          printf(" flags=0x%02x", mi->mode_flags);
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
      if (n == sizeof(s))
        {
          print_sample(&s);
          count++;
        }

      struct timespec t1;
      clock_gettime(CLOCK_MONOTONIC, &t1);
      long elapsed_ms = (t1.tv_sec - t0.tv_sec) * 1000 +
                        (t1.tv_nsec - t0.tv_nsec) / 1000000;
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
          "  sensor                                 list all class topics\n"
          "  sensor list                            same as above\n"
          "  sensor <class>                         status one-liner\n"
          "  sensor <class> info                    device info / mode schema\n"
          "  sensor <class> status                  engine + traffic counters\n"
          "  sensor <class> watch [ms]              decode samples (default 1000)\n"
          "  sensor <class> fps [ms]                rate-only count, no per-sample print\n"
          "  sensor <class> select <mode>           SELECT mode\n"
          "  sensor <class> send <mode> <hex>...    SEND writable-mode payload\n"
          "  sensor <class> pwm <ch0> [ch1 ch2 ch3] LED brightness / motor duty\n"
          "  <class> ::= color | ultrasonic | force | motor_m | motor_r | motor_l\n");
}

int main(int argc, FAR char *argv[])
{
  if (argc < 2 || strcmp(argv[1], "list") == 0)
    {
      return do_list() < 0 ? 1 : 0;
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
      return do_status_one(c, false) < 0 ? 1 : 0;
    }

  const char *verb = argv[2];

  if (strcmp(verb, "info") == 0)
    {
      return do_info(c) < 0 ? 1 : 0;
    }

  if (strcmp(verb, "status") == 0)
    {
      return do_status_one(c, false) < 0 ? 1 : 0;
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

  usage();
  return 1;
}
