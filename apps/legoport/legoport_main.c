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
