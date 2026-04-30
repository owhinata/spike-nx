/****************************************************************************
 * apps/legosensor/legosensor_main.c
 *
 * CLI for the SPIKE Prime Hub LEGO sensor uORB driver (Issue #45).
 *
 * Usage:
 *   legosensor                     - same as `legosensor list`
 *   legosensor list                - one-line per port: type / mode / state
 *   legosensor info <N>            - full lump_device_info_s dump (modes 0..n)
 *   legosensor mode <N> <m>        - CLAIM, SELECT, poll for confirmation
 *   legosensor send <N> <m> <hex>  - CLAIM, SEND writable-mode payload
 *   legosensor watch <N> [<ms>]    - poll + read decoded samples
 *   legosensor claim <N>           - explicit CLAIM (held until close)
 *   legosensor release <N>         - explicit RELEASE
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

#define DEVPATH_FMT      "/dev/uorb/sensor_lego%d"
#define SELECT_POLL_MS   500
#define WATCH_DEFAULT_MS 1000

/****************************************************************************
 * Private Functions — helpers
 ****************************************************************************/

static int build_devpath(int port, char *out, size_t outlen)
{
  if (port < 0 || port >= LEGOSENSOR_NUM_PORTS)
    {
      return -EINVAL;
    }

  snprintf(out, outlen, DEVPATH_FMT, port);
  return 0;
}

static int parse_port(const char *s)
{
  if (s == NULL || s[0] == '\0' || s[1] != '\0')
    {
      return -EINVAL;
    }

  if (s[0] >= 'A' && s[0] <= 'F')
    {
      return s[0] - 'A';
    }

  if (s[0] >= 'a' && s[0] <= 'f')
    {
      return s[0] - 'a';
    }

  if (s[0] >= '0' && s[0] <= '5')
    {
      return s[0] - '0';
    }

  return -EINVAL;
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

/****************************************************************************
 * Private Functions — sample decoding
 ****************************************************************************/

static void print_sample(const struct lump_sample_s *s)
{
  printf("[%c] gen=%" PRIu32 " seq=%" PRIu32 " mode=%u (%s x%u) ",
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

static int do_list(void)
{
  char path[32];

  printf("Port  Type  State    Mode  Gen  RX(B)     TX(B)\n");
  printf("----  ----  -------  ----  ---  --------  --------\n");

  for (int p = 0; p < LEGOSENSOR_NUM_PORTS; p++)
    {
      build_devpath(p, path, sizeof(path));

      int fd = open(path, O_RDONLY | O_NONBLOCK);
      if (fd < 0)
        {
          printf("  %c   <open: %d>\n", 'A' + p, errno);
          continue;
        }

      struct lump_status_full_s st;
      memset(&st, 0, sizeof(st));
      int r = ioctl(fd, LEGOSENSOR_GET_STATUS, (unsigned long)&st);
      close(fd);

      if (r < 0)
        {
          printf("  %c   <ioctl: %d>\n", 'A' + p, errno);
          continue;
        }

      printf("  %c    %3u  %-7s   %3u  %3u  %8" PRIu32 "  %8" PRIu32 "\n",
             'A' + p, st.type_id, state_name(st.state),
             st.current_mode, 0u, st.rx_bytes, st.tx_bytes);
    }

  return 0;
}

static int do_info(int port)
{
  char path[32];
  int ret;

  build_devpath(port, path, sizeof(path));

  int fd = open(path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s): %s\n", path, strerror(errno));
      return -errno;
    }

  struct lump_device_info_s info;
  memset(&info, 0, sizeof(info));
  ret = ioctl(fd, LEGOSENSOR_GET_INFO, (unsigned long)&info);
  close(fd);

  if (ret < 0)
    {
      fprintf(stderr, "ioctl GET_INFO: %s\n", strerror(errno));
      return -errno;
    }

  printf("Port %c\n", 'A' + port);
  printf("  type_id     : %u\n", info.type_id);
  printf("  num_modes   : %u\n", info.num_modes);
  printf("  current_mode: %u\n", info.current_mode);
  printf("  baud        : %" PRIu32 "\n", info.baud);
  printf("  fw / hw     : 0x%08" PRIx32 " / 0x%08" PRIx32 "\n",
         info.fw_version, info.hw_version);
  printf("  flags       : 0x%02x%s%s%s\n", info.flags,
         (info.flags & LUMP_FLAG_SYNCED)  ? " SYNCED"  : "",
         (info.flags & LUMP_FLAG_DATA_OK) ? " DATA_OK" : "",
         (info.flags & LUMP_FLAG_ERROR)   ? " ERROR"   : "");

  for (int m = 0; m < info.num_modes && m < LUMP_MAX_MODES; m++)
    {
      const struct lump_mode_info_s *mi = &info.modes[m];
      printf("  mode[%d] %-12s  %ux%s  raw[%g..%g] pct[%g..%g] si[%g..%g] %s%s\n",
             m, mi->name, mi->num_values, dtype_name(mi->data_type),
             (double)mi->raw_min, (double)mi->raw_max,
             (double)mi->pct_min, (double)mi->pct_max,
             (double)mi->si_min,  (double)mi->si_max,
             mi->units, mi->writable ? " writable" : "");
    }

  return 0;
}

static int do_mode(int port, uint8_t mode)
{
  char path[32];
  int  ret;

  build_devpath(port, path, sizeof(path));

  int fd = open(path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s): %s\n", path, strerror(errno));
      return -errno;
    }

  ret = ioctl(fd, LEGOSENSOR_CLAIM, 0);
  if (ret < 0)
    {
      fprintf(stderr, "CLAIM: %s\n", strerror(errno));
      close(fd);
      return -errno;
    }

  ret = ioctl(fd, LEGOSENSOR_SELECT, (unsigned long)mode);
  if (ret < 0)
    {
      fprintf(stderr, "SELECT: %s\n", strerror(errno));
      ioctl(fd, LEGOSENSOR_RELEASE, 0);
      close(fd);
      return -errno;
    }

  /* Wait up to SELECT_POLL_MS for a sample whose mode matches. */

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

  ioctl(fd, LEGOSENSOR_RELEASE, 0);
  close(fd);

  if (!confirmed)
    {
      fprintf(stderr, "SELECT timeout (mode %u not confirmed in %d ms)\n",
              mode, SELECT_POLL_MS);
      return -ETIMEDOUT;
    }

  printf("Port %c switched to mode %u\n", 'A' + port, mode);
  return 0;
}

static int do_send(int port, uint8_t mode, int argc, char **argv)
{
  char path[32];
  int  ret;

  if (argc <= 0)
    {
      fprintf(stderr, "usage: legosensor send <N> <m> <hex>...\n");
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
      ret = parse_hex_byte(argv[i], &snd.data[i]);
      if (ret < 0)
        {
          fprintf(stderr, "bad hex byte: %s\n", argv[i]);
          return -EINVAL;
        }
    }

  build_devpath(port, path, sizeof(path));
  int fd = open(path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s): %s\n", path, strerror(errno));
      return -errno;
    }

  ret = ioctl(fd, LEGOSENSOR_CLAIM, 0);
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

  ioctl(fd, LEGOSENSOR_RELEASE, 0);
  close(fd);

  return ret < 0 ? -errno : 0;
}

static int do_watch(int port, int duration_ms)
{
  char path[32];

  build_devpath(port, path, sizeof(path));
  int fd = open(path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s): %s\n", path, strerror(errno));
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

/* CLAIM / RELEASE that hold the fd open as long as the CLI process
 * runs.  Useful for chaining multiple SELECT/SEND from a script.
 */

static int do_claim(int port, bool acquire)
{
  char path[32];
  int  ret;

  build_devpath(port, path, sizeof(path));

  int fd = open(path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s): %s\n", path, strerror(errno));
      return -errno;
    }

  ret = ioctl(fd, acquire ? LEGOSENSOR_CLAIM : LEGOSENSOR_RELEASE, 0);
  if (ret < 0)
    {
      fprintf(stderr, "%s: %s\n",
              acquire ? "CLAIM" : "RELEASE", strerror(errno));
    }

  close(fd);
  return ret < 0 ? -errno : 0;
}

/****************************************************************************
 * Public Function: legosensor_main
 ****************************************************************************/

static void usage(void)
{
  fprintf(stderr,
          "usage:\n"
          "  legosensor                     list all ports (default)\n"
          "  legosensor list                same as above\n"
          "  legosensor info <N>            mode schema dump\n"
          "  legosensor mode <N> <m>        CLAIM + SELECT mode + RELEASE\n"
          "  legosensor send <N> <m> <hex>...  CLAIM + SEND + RELEASE\n"
          "  legosensor watch <N> [ms]      poll + decode samples (default 1000 ms)\n"
          "  legosensor claim <N>           CLAIM (released on fd close)\n"
          "  legosensor release <N>         RELEASE\n"
          "  N is 0..5 or A..F\n");
}

int main(int argc, FAR char *argv[])
{
  if (argc < 2 || strcmp(argv[1], "list") == 0)
    {
      return do_list() < 0 ? 1 : 0;
    }

  if (strcmp(argv[1], "info") == 0)
    {
      if (argc < 3)
        {
          usage();
          return 1;
        }
      int p = parse_port(argv[2]);
      if (p < 0)
        {
          usage();
          return 1;
        }
      return do_info(p) < 0 ? 1 : 0;
    }

  if (strcmp(argv[1], "mode") == 0)
    {
      if (argc < 4)
        {
          usage();
          return 1;
        }
      int p = parse_port(argv[2]);
      if (p < 0)
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
      return do_mode(p, (uint8_t)m) < 0 ? 1 : 0;
    }

  if (strcmp(argv[1], "send") == 0)
    {
      if (argc < 5)
        {
          usage();
          return 1;
        }
      int p = parse_port(argv[2]);
      if (p < 0)
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
      return do_send(p, (uint8_t)m, argc - 4, &argv[4]) < 0 ? 1 : 0;
    }

  if (strcmp(argv[1], "watch") == 0)
    {
      if (argc < 3)
        {
          usage();
          return 1;
        }
      int p = parse_port(argv[2]);
      if (p < 0)
        {
          usage();
          return 1;
        }
      int ms = WATCH_DEFAULT_MS;
      if (argc >= 4)
        {
          ms = atoi(argv[3]);
          if (ms <= 0)
            {
              ms = WATCH_DEFAULT_MS;
            }
        }
      return do_watch(p, ms) < 0 ? 1 : 0;
    }

  if (strcmp(argv[1], "claim") == 0 || strcmp(argv[1], "release") == 0)
    {
      if (argc < 3)
        {
          usage();
          return 1;
        }
      int p = parse_port(argv[2]);
      if (p < 0)
        {
          usage();
          return 1;
        }
      bool acquire = (argv[1][0] == 'c');
      return do_claim(p, acquire) < 0 ? 1 : 0;
    }

  usage();
  return 1;
}
