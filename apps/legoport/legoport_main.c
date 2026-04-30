/****************************************************************************
 * apps/legoport/legoport_main.c
 *
 * CLI utility for the SPIKE Prime Hub I/O port DCM (Issue #42) and the
 * LUMP UART engine diagnostics (Issue #43, gated by CONFIG_LEGO_LUMP_DIAG).
 *
 * Usage:
 *   legoport             - same as `legoport list`
 *   legoport list        - dump all 6 ports' detected type + flags
 *   legoport info <N>    - verbose single-port view (N = 0..5)
 *   legoport wait <N> [timeout_ms]
 *                        - block on connect (timeout_ms=0: infinite)
 *   legoport stats       - HPWORK cadence stats (max step / max interval)
 *   legoport lump-hw dump
 *                        - dump RCC / USART / NVIC state for the 6 LUMP
 *                          UARTs (CONFIG_LEGO_LUMP_DIAG=y only)
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <arch/board/board_legoport.h>

#ifdef CONFIG_LEGO_LUMP
#  include <arch/board/board_lump.h>
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char * const g_type_names[] =
{
  [LEGOPORT_TYPE_NONE]              = "NONE",
  [LEGOPORT_TYPE_LPF2_MMOTOR]       = "MMOTOR",
  [LEGOPORT_TYPE_LPF2_TRAIN]        = "TRAIN",
  [LEGOPORT_TYPE_LPF2_TURN]         = "TURN",
  [LEGOPORT_TYPE_LPF2_POWER]        = "POWER",
  [LEGOPORT_TYPE_LPF2_TOUCH]        = "TOUCH",
  [LEGOPORT_TYPE_LPF2_LMOTOR]       = "LMOTOR",
  [LEGOPORT_TYPE_LPF2_XMOTOR]       = "XMOTOR",
  [LEGOPORT_TYPE_LPF2_LIGHT]        = "LIGHT",
  [LEGOPORT_TYPE_LPF2_LIGHT1]       = "LIGHT1",
  [LEGOPORT_TYPE_LPF2_LIGHT2]       = "LIGHT2",
  [LEGOPORT_TYPE_LPF2_TPOINT]       = "TPOINT",
  [LEGOPORT_TYPE_LPF2_EXPLOD]       = "EXPLOD",
  [LEGOPORT_TYPE_LPF2_3_PART]       = "3_PART",
  [LEGOPORT_TYPE_LPF2_UNKNOWN_UART] = "UNKNOWN_UART",
};

#define NUM_TYPE_NAMES (sizeof(g_type_names) / sizeof(g_type_names[0]))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *type_name(uint8_t t)
{
  if (t < NUM_TYPE_NAMES && g_type_names[t] != NULL)
    {
      return g_type_names[t];
    }
  return "?";
}

static int build_devpath(int port, char *out, size_t outlen)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      return -EINVAL;
    }
  snprintf(out, outlen, BOARD_LEGOPORT_DEVPATH_FMT, port);
  return 0;
}

static void print_flags(uint8_t flags)
{
  printf("[");
  if (flags & LEGOPORT_FLAG_CONNECTED)  printf("CONNECTED ");
  if (flags & LEGOPORT_FLAG_IS_UART)    printf("UART ");
  if (flags & LEGOPORT_FLAG_IS_PASSIVE) printf("PASSIVE ");
  if (flags & LEGOPORT_FLAG_HANDOFF_OK) printf("HANDOFF_OK ");
  printf("]");
}

static int do_list(void)
{
  char path[24];
  printf("Port  Type             Flags\n");
  printf("----  ---------------  -----\n");

  for (int p = 0; p < BOARD_LEGOPORT_COUNT; p++)
    {
      build_devpath(p, path, sizeof(path));
      int fd = open(path, O_RDONLY);
      if (fd < 0)
        {
          printf("  %c   <open: %d>\n", 'A' + p, errno);
          continue;
        }

      struct legoport_info_s info;
      if (ioctl(fd, LEGOPORT_GET_DEVICE_INFO, (unsigned long)&info) < 0)
        {
          printf("  %c   <ioctl: %d>\n", 'A' + p, errno);
        }
      else
        {
          printf("  %c   %-15s  ", 'A' + p, type_name(info.device_type));
          print_flags(info.flags);
          printf(" #%lu\n", (unsigned long)info.event_counter);
        }
      close(fd);
    }
  return 0;
}

static int do_info(int port)
{
  char path[24];
  if (build_devpath(port, path, sizeof(path)) < 0)
    {
      printf("invalid port: %d (must be 0..%d)\n", port,
             BOARD_LEGOPORT_COUNT - 1);
      return 1;
    }

  int fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      printf("cannot open %s: %d\n", path, errno);
      return 1;
    }

  struct legoport_info_s info;
  if (ioctl(fd, LEGOPORT_GET_DEVICE_INFO, (unsigned long)&info) < 0)
    {
      printf("ioctl GET_DEVICE_INFO failed: %d\n", errno);
      close(fd);
      return 1;
    }

  printf("Port %c (%s):\n", 'A' + port, path);
  printf("  device_type:   %u (%s)\n", info.device_type,
         type_name(info.device_type));
  printf("  flags:         0x%02x ", info.flags);
  print_flags(info.flags);
  printf("\n");
  printf("  event_counter: %lu\n", (unsigned long)info.event_counter);

  close(fd);
  return 0;
}

static int do_wait(int port, uint32_t timeout_ms)
{
  char path[24];
  if (build_devpath(port, path, sizeof(path)) < 0)
    {
      printf("invalid port: %d\n", port);
      return 1;
    }

  int fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      printf("cannot open %s: %d\n", path, errno);
      return 1;
    }

  /* Snapshot current state into the fd so WAIT blocks on the next edge. */

  struct legoport_info_s info;
  if (ioctl(fd, LEGOPORT_GET_DEVICE_INFO, (unsigned long)&info) < 0)
    {
      printf("snapshot failed: %d\n", errno);
      close(fd);
      return 1;
    }

  printf("Port %c (%s) waiting for connect (current=%s, timeout=%lums)...\n",
         'A' + port, path, type_name(info.device_type),
         (unsigned long)timeout_ms);

  int ret = ioctl(fd, LEGOPORT_WAIT_CONNECT, (unsigned long)timeout_ms);
  if (ret < 0)
    {
      if (errno == ETIMEDOUT)
        {
          printf("timeout\n");
          close(fd);
          return 2;
        }
      printf("wait failed: %d\n", errno);
      close(fd);
      return 1;
    }

  if (ioctl(fd, LEGOPORT_GET_DEVICE_INFO, (unsigned long)&info) == 0)
    {
      printf("connected: %s ", type_name(info.device_type));
      print_flags(info.flags);
      printf("\n");
    }

  close(fd);
  return 0;
}

static int do_stats(bool reset)
{
  /* Stats live on every port, but the values are global; query port 0. */

  char path[24];
  build_devpath(0, path, sizeof(path));

  int fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      printf("cannot open %s: %d\n", path, errno);
      return 1;
    }

  if (reset)
    {
      if (ioctl(fd, LEGOPORT_RESET_STATS, 0) < 0)
        {
          printf("ioctl RESET_STATS failed: %d\n", errno);
          close(fd);
          return 1;
        }
      printf("stats cleared\n");
      close(fd);
      return 0;
    }

  struct legoport_stats_s stats;
  if (ioctl(fd, LEGOPORT_GET_STATS, (unsigned long)&stats) < 0)
    {
      printf("ioctl GET_STATS failed: %d\n", errno);
      close(fd);
      return 1;
    }

  printf("HPWORK DCM stats:\n");
  printf("  total invocations: %lu\n",
         (unsigned long)stats.total_invocations);
  printf("  max step:          %lu us\n",
         (unsigned long)stats.max_step_us);
  printf("  max interval:      %lu us  (target: 2000 us)\n",
         (unsigned long)stats.max_interval_us);
  printf("  intervals > 4ms:   %lu\n",   (unsigned long)stats.late_4ms);
  printf("  intervals > 10ms:  %lu\n",   (unsigned long)stats.late_10ms);
  printf("  intervals > 100ms: %lu\n",   (unsigned long)stats.late_100ms);
  close(fd);
  return 0;
}

#ifdef CONFIG_LEGO_LUMP
static const char * const g_lump_data_type_names[] =
{
  [LUMP_DATA_INT8]  = "INT8",
  [LUMP_DATA_INT16] = "INT16",
  [LUMP_DATA_INT32] = "INT32",
  [LUMP_DATA_FLOAT] = "FLOAT",
};

static const char *lump_data_type_name(uint8_t t)
{
  if (t < sizeof(g_lump_data_type_names) /
          sizeof(g_lump_data_type_names[0]))
    {
      return g_lump_data_type_names[t];
    }
  return "?";
}

static int do_lump_info(int port)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      printf("invalid port: %d\n", port);
      return 1;
    }

  char path[24];
  build_devpath(port, path, sizeof(path));
  int fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      printf("cannot open %s: %d\n", path, errno);
      return 1;
    }

  struct lump_device_info_s info;
  if (ioctl(fd, LEGOPORT_LUMP_GET_INFO, (unsigned long)&info) < 0)
    {
      if (errno == EAGAIN)
        {
          printf("Port %c: not synced yet (engine in SYNCING/INFO/IDLE)\n",
                 'A' + port);
        }
      else
        {
          printf("ioctl LUMP_GET_INFO failed: %d\n", errno);
        }
      close(fd);
      return 1;
    }
  close(fd);

  printf("Port %c LUMP info:\n", 'A' + port);
  printf("  type_id:      %u\n", info.type_id);
  printf("  num_modes:    %u\n", info.num_modes);
  printf("  current_mode: %u\n", info.current_mode);
  printf("  flags:        0x%02x %s%s%s\n",
         info.flags,
         (info.flags & LUMP_FLAG_SYNCED) ? "[SYNCED]" : "",
         (info.flags & LUMP_FLAG_DATA_OK) ? "[DATA_OK]" : "",
         (info.flags & LUMP_FLAG_ERROR) ? "[ERROR]" : "");
  printf("  baud:         %lu\n", (unsigned long)info.baud);
  if (info.fw_version || info.hw_version)
    {
      printf("  fw_version:   0x%08lx\n", (unsigned long)info.fw_version);
      printf("  hw_version:   0x%08lx\n", (unsigned long)info.hw_version);
    }

  for (uint8_t m = 0; m < info.num_modes && m < LUMP_MAX_MODES; m++)
    {
      const struct lump_mode_info_s *mi = &info.modes[m];
      printf("  mode %u  %-12s  %u x %s%s",
             m,
             mi->name[0] ? mi->name : "?",
             mi->num_values,
             lump_data_type_name(mi->data_type),
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
      printf("\n");
    }

  return 0;
}
#endif

static const char *lump_state_name(uint8_t s)
{
  static const char * const names[] =
  {
    [LUMP_ENGINE_IDLE]    = "IDLE",
    [LUMP_ENGINE_SYNCING] = "SYNCING",
    [LUMP_ENGINE_INFO]    = "INFO",
    [LUMP_ENGINE_DATA]    = "DATA",
    [LUMP_ENGINE_ERR]     = "ERR",
  };
  if (s < sizeof(names) / sizeof(names[0]) && names[s])
    {
      return names[s];
    }
  return "?";
}

static int do_lump_status(void)
{
  printf("Port  State    Type  Mode  Baud    RX(B)     TX(B)   "
         "Drops  Backoff  StkHWM\n");
  printf("----  -------  ----  ----  ------  --------  ------  "
         "-----  -------  ----------\n");

  for (int p = 0; p < BOARD_LEGOPORT_COUNT; p++)
    {
      char path[24];
      build_devpath(p, path, sizeof(path));
      int fd = open(path, O_RDONLY);
      if (fd < 0)
        {
          printf("  %c   <open: %d>\n", 'A' + p, errno);
          continue;
        }

      struct lump_status_full_s s;
      if (ioctl(fd, LEGOPORT_LUMP_GET_STATUS_EX, (unsigned long)&s) < 0)
        {
          printf("  %c   <ioctl: %d>\n", 'A' + p, errno);
          close(fd);
          continue;
        }
      close(fd);

      printf("  %c   %-7s  %3u   %3u   %6lu  %8lu  %6lu  %5lu  %7lu  "
             "%4lu/%4lu\n",
             'A' + p,
             lump_state_name(s.state),
             s.type_id,
             s.current_mode,
             (unsigned long)s.baud,
             (unsigned long)s.rx_bytes,
             (unsigned long)s.tx_bytes,
             (unsigned long)s.dq_dropped,
             (unsigned long)s.backoff_step,
             (unsigned long)s.stk_used,
             (unsigned long)s.stk_size);
    }

  return 0;
}

static int do_lump_set_mode(int port, int mode)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      printf("invalid port: %d\n", port);
      return 1;
    }

  char path[24];
  build_devpath(port, path, sizeof(path));
  int fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      printf("cannot open %s: %d\n", path, errno);
      return 1;
    }

  int rc = ioctl(fd, LEGOPORT_LUMP_SELECT, (unsigned long)mode);
  close(fd);
  if (rc < 0)
    {
      printf("ioctl LUMP_SELECT failed: %d\n", errno);
      return 1;
    }
  printf("Port %c: mode select %d queued\n", 'A' + port, mode);
  return 0;
}

static int parse_hex_byte(const char *s, uint8_t *out)
{
  if (s[0] == '\0' || s[1] == '\0' || s[2] != '\0')
    {
      return -1;
    }
  char buf[3] = { s[0], s[1], 0 };
  char *end = NULL;
  long v = strtol(buf, &end, 16);
  if (end == NULL || *end != 0 || v < 0 || v > 0xff)
    {
      return -1;
    }
  *out = (uint8_t)v;
  return 0;
}

static int do_lump_send(int port, int mode, int argc, FAR char *argv[])
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      printf("invalid port: %d\n", port);
      return 1;
    }
  if (mode < 0 || mode > 7)
    {
      printf("invalid mode: %d (0..7)\n", mode);
      return 1;
    }
  if (argc < 1 || argc > 32)
    {
      printf("data byte count must be 1..32, got %d\n", argc);
      return 1;
    }

  struct legoport_lump_send_arg_s req;
  memset(&req, 0, sizeof(req));
  req.mode = (uint8_t)mode;
  req.len  = (uint8_t)argc;
  for (int i = 0; i < argc; i++)
    {
      if (parse_hex_byte(argv[i], &req.data[i]) < 0)
        {
          printf("bad hex byte: %s (need 2 hex chars)\n", argv[i]);
          return 1;
        }
    }

  char path[24];
  build_devpath(port, path, sizeof(path));
  int fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      printf("cannot open %s: %d\n", path, errno);
      return 1;
    }

  int rc = ioctl(fd, LEGOPORT_LUMP_SEND, (unsigned long)&req);
  close(fd);
  if (rc < 0)
    {
      printf("ioctl LUMP_SEND failed: %d\n", errno);
      return 1;
    }
  printf("Port %c: send mode=%u %u bytes queued\n",
         'A' + port, req.mode, req.len);
  return 0;
}

static int do_lump_watch(int port, int duration_ms)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      printf("invalid port: %d\n", port);
      return 1;
    }
  if (duration_ms <= 0 || duration_ms > 60000)
    {
      printf("duration must be 1..60000 ms\n");
      return 1;
    }

  char path[24];
  build_devpath(port, path, sizeof(path));
  int fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      printf("cannot open %s: %d\n", path, errno);
      return 1;
    }

  printf("Port %c watching %d ms... (Ctrl-C to stop)\n",
         'A' + port, duration_ms);

  /* Poll the engine's DATA ring at ~10 ms cadence. */

  struct timespec start;
  clock_gettime(CLOCK_MONOTONIC, &start);

  uint32_t frames = 0;
  for (;;)
    {
      struct timespec now;
      clock_gettime(CLOCK_MONOTONIC, &now);
      long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000L +
                       (now.tv_nsec - start.tv_nsec) / 1000000L;
      if (elapsed_ms >= duration_ms)
        {
          break;
        }

      struct lump_data_frame_s frame;
      int rc = ioctl(fd, LEGOPORT_LUMP_POLL_DATA, (unsigned long)&frame);
      if (rc == 0)
        {
          frames++;
          printf("[%c] mode=%u len=%u",
                 'A' + port, frame.mode, frame.len);
          for (uint8_t i = 0; i < frame.len; i++)
            {
              printf(" %02x", frame.data[i]);
            }
          printf("\n");
        }
      else if (errno != EAGAIN)
        {
          printf("ioctl LUMP_POLL_DATA failed: %d\n", errno);
          close(fd);
          return 1;
        }

      usleep(10 * 1000);
    }

  close(fd);
  printf("Port %c: %lu frames in %d ms\n",
         'A' + port, (unsigned long)frames, duration_ms);
  return 0;
}

#ifdef CONFIG_LEGO_LUMP_DIAG
static int do_lump_hw_dump(void)
{
  /* Any port works — the dump ioctl is per-engine, not per-port. */

  char path[24];
  build_devpath(0, path, sizeof(path));
  int fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      printf("cannot open %s: %d\n", path, errno);
      return 1;
    }

  if (ioctl(fd, LEGOPORT_LUMP_HW_DUMP, 0) < 0)
    {
      printf("ioctl LUMP_HW_DUMP failed: %d\n", errno);
      close(fd);
      return 1;
    }

  printf("lump-hw: dump emitted to syslog (use `dmesg` to view)\n");
  close(fd);
  return 0;
}
#endif

static void usage(void)
{
  printf("Usage:\n");
  printf("  legoport             - alias of `legoport list`\n");
  printf("  legoport list        - dump all 6 ports\n");
  printf("  legoport info <N>    - single-port info (N = 0..5)\n");
  printf("  legoport wait <N> [timeout_ms]\n"
         "                       - block on connect edge\n");
  printf("  legoport stats       - HPWORK cadence stats\n");
  printf("  legoport stats reset - clear max_step_us / max_interval_us\n");
#ifdef CONFIG_LEGO_LUMP
  printf("  legoport lump status     - per-port engine state table\n");
  printf("  legoport lump info <N>\n"
         "                       - dump LUMP device info (post-SYNC)\n");
  printf("  legoport lump set-mode <N> <m>\n"
         "                       - request mode switch (CMD SELECT)\n");
  printf("  legoport lump send <N> <m> <hex>...\n"
         "                       - send DATA frame (writable mode)\n");
  printf("  legoport lump watch <N> <ms>\n"
         "                       - dump DATA frames for `ms` ms\n");
#endif
#ifdef CONFIG_LEGO_LUMP_DIAG
  printf("  legoport lump-hw dump\n"
         "                       - dump RCC/USART/NVIC for 6 LUMP UARTs\n");
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  if (argc < 2 || strcmp(argv[1], "list") == 0)
    {
      return do_list();
    }

  if (strcmp(argv[1], "info") == 0)
    {
      if (argc < 3)
        {
          usage();
          return 1;
        }
      return do_info(atoi(argv[2]));
    }

  if (strcmp(argv[1], "wait") == 0)
    {
      if (argc < 3)
        {
          usage();
          return 1;
        }
      uint32_t timeout = (argc >= 4) ? (uint32_t)atoi(argv[3]) : 0;
      return do_wait(atoi(argv[2]), timeout);
    }

  if (strcmp(argv[1], "stats") == 0)
    {
      bool reset = (argc >= 3 && strcmp(argv[2], "reset") == 0);
      return do_stats(reset);
    }

#ifdef CONFIG_LEGO_LUMP
  if (strcmp(argv[1], "lump") == 0)
    {
      if (argc >= 3 && strcmp(argv[2], "status") == 0)
        {
          return do_lump_status();
        }
      if (argc >= 3 && strcmp(argv[2], "info") == 0)
        {
          if (argc < 4)
            {
              usage();
              return 1;
            }
          return do_lump_info(atoi(argv[3]));
        }
      if (argc >= 3 && strcmp(argv[2], "set-mode") == 0)
        {
          if (argc < 5)
            {
              usage();
              return 1;
            }
          return do_lump_set_mode(atoi(argv[3]), atoi(argv[4]));
        }
      if (argc >= 3 && strcmp(argv[2], "send") == 0)
        {
          if (argc < 6)
            {
              usage();
              return 1;
            }
          return do_lump_send(atoi(argv[3]), atoi(argv[4]),
                              argc - 5, &argv[5]);
        }
      if (argc >= 3 && strcmp(argv[2], "watch") == 0)
        {
          if (argc < 5)
            {
              usage();
              return 1;
            }
          return do_lump_watch(atoi(argv[3]), atoi(argv[4]));
        }
      usage();
      return 1;
    }
#endif

#ifdef CONFIG_LEGO_LUMP_DIAG
  if (strcmp(argv[1], "lump-hw") == 0)
    {
      if (argc < 3 || strcmp(argv[2], "dump") != 0)
        {
          usage();
          return 1;
        }
      return do_lump_hw_dump();
    }
#endif

  usage();
  return 1;
}
