/****************************************************************************
 * apps/battery/battery_main.c
 *
 * Battery test utility for SPIKE Prime Hub.
 * Reads battery gauge and charger status via NuttX battery IOCTL.
 *
 * Usage:
 *   battery           - Show gauge + charger info
 *   battery gauge     - Show gauge info only
 *   battery charger   - Show charger info only
 *   battery monitor [N] - Monitor N times at 1s interval (default: 10)
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <nuttx/power/battery_ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BAT_GAUGE_DEV    "/dev/bat0"
#define BAT_CHARGER_DEV  "/dev/charge0"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *state_str(int state)
{
  switch (state)
    {
      case BATTERY_UNKNOWN:     return "UNKNOWN";
      case BATTERY_FAULT:       return "FAULT";
      case BATTERY_IDLE:        return "IDLE";
      case BATTERY_FULL:        return "FULL";
      case BATTERY_CHARGING:    return "CHARGING";
      case BATTERY_DISCHARGING: return "DISCHARGING";
      default:                  return "???";
    }
}

static const char *health_str(int health)
{
  switch (health)
    {
      case BATTERY_HEALTH_UNKNOWN:       return "UNKNOWN";
      case BATTERY_HEALTH_GOOD:          return "GOOD";
      case BATTERY_HEALTH_DEAD:          return "DEAD";
      case BATTERY_HEALTH_OVERHEAT:      return "OVERHEAT";
      case BATTERY_HEALTH_OVERVOLTAGE:   return "OVERVOLTAGE";
      case BATTERY_HEALTH_UNDERVOLTAGE:  return "UNDERVOLTAGE";
      case BATTERY_HEALTH_OVERCURRENT:   return "OVERCURRENT";
      case BATTERY_HEALTH_UNSPEC_FAIL:   return "UNSPEC_FAIL";
      default:                           return "???";
    }
}

static void show_gauge(void)
{
  int fd;
  int val;
  bool online;

  fd = open(BAT_GAUGE_DEV, O_RDONLY);
  if (fd < 0)
    {
      printf("gauge: cannot open %s: %d\n", BAT_GAUGE_DEV, errno);
      return;
    }

  printf("=== Battery Gauge (%s) ===\n", BAT_GAUGE_DEV);

  if (ioctl(fd, BATIOC_STATE, &val) == 0)
    {
      printf("  State:       %s\n", state_str(val));
    }

  if (ioctl(fd, BATIOC_ONLINE, &online) == 0)
    {
      printf("  Online:      %s\n", online ? "yes" : "no");
    }

  if (ioctl(fd, BATIOC_VOLTAGE, &val) == 0)
    {
      printf("  Voltage:     %d mV\n", val);
    }

  if (ioctl(fd, BATIOC_CURRENT, &val) == 0)
    {
      printf("  Current:     %d mA\n", val);
    }

  if (ioctl(fd, BATIOC_CAPACITY, &val) == 0)
    {
      printf("  Capacity:    %d %%\n", val);
    }

  if (ioctl(fd, BATIOC_TEMPERATURE, &val) == 0)
    {
      printf("  Temperature: %d.%03d C\n", val / 1000, val % 1000);
    }
  else
    {
      printf("  Temperature: N/A\n");
    }

  close(fd);
}

static void show_charger(void)
{
  int fd;
  int val;
  bool online;
  unsigned int chipid;

  fd = open(BAT_CHARGER_DEV, O_RDONLY);
  if (fd < 0)
    {
      printf("charger: cannot open %s: %d\n", BAT_CHARGER_DEV, errno);
      return;
    }

  printf("=== Battery Charger (%s) ===\n", BAT_CHARGER_DEV);

  if (ioctl(fd, BATIOC_STATE, &val) == 0)
    {
      printf("  State:       %s\n", state_str(val));
    }

  if (ioctl(fd, BATIOC_HEALTH, &val) == 0)
    {
      printf("  Health:      %s\n", health_str(val));
    }

  if (ioctl(fd, BATIOC_ONLINE, &online) == 0)
    {
      printf("  USB online:  %s\n", online ? "yes" : "no");
    }

  if (ioctl(fd, BATIOC_CHIPID, &chipid) == 0)
    {
      printf("  Chip ID:     0x%04x\n", chipid);
    }

  if (ioctl(fd, BATIOC_VOLTAGE_INFO, &val) == 0)
    {
      printf("  Target V:    %d mV\n", val);
    }

  close(fd);
}

static void monitor(int count)
{
  int gfd;
  int cfd;
  int voltage;
  int current;
  int capacity;
  int temp;
  int gstate;
  int cstate;
  bool usb;
  int i;

  gfd = open(BAT_GAUGE_DEV, O_RDONLY);
  if (gfd < 0)
    {
      printf("cannot open %s: %d\n", BAT_GAUGE_DEV, errno);
      return;
    }

  cfd = open(BAT_CHARGER_DEV, O_RDONLY);
  if (cfd < 0)
    {
      printf("cannot open %s: %d\n", BAT_CHARGER_DEV, errno);
      close(gfd);
      return;
    }

  printf("%-12s %7s %7s %5s %7s %-12s %s\n",
         "GAUGE", "mV", "mA", "SoC", "Temp", "CHARGER", "USB");
  printf("%-12s %7s %7s %5s %7s %-12s %s\n",
         "------------", "-------", "-------", "-----",
         "-------", "------------", "---");

  for (i = 0; i < count; i++)
    {
      ioctl(gfd, BATIOC_STATE, &gstate);
      ioctl(gfd, BATIOC_VOLTAGE, &voltage);
      ioctl(gfd, BATIOC_CURRENT, &current);
      ioctl(gfd, BATIOC_CAPACITY, &capacity);
      ioctl(gfd, BATIOC_TEMPERATURE, &temp);
      ioctl(cfd, BATIOC_STATE, &cstate);
      ioctl(cfd, BATIOC_ONLINE, &usb);

      printf("%-12s %7d %7d %4d%% %3d.%01dC %-12s %s\n",
             state_str(gstate),
             voltage, current, capacity,
             temp / 1000, (temp % 1000) / 100,
             state_str(cstate),
             usb ? "yes" : "no");

      if (i < count - 1)
        {
          sleep(1);
        }
    }

  close(cfd);
  close(gfd);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      show_gauge();
      printf("\n");
      show_charger();
      return 0;
    }

  if (strcmp(argv[1], "gauge") == 0)
    {
      show_gauge();
    }
  else if (strcmp(argv[1], "charger") == 0)
    {
      show_charger();
    }
  else if (strcmp(argv[1], "monitor") == 0)
    {
      int count = 10;
      if (argc >= 3)
        {
          count = atoi(argv[2]);
          if (count <= 0)
            {
              count = 10;
            }
        }

      monitor(count);
    }
  else
    {
      printf("Usage: battery [gauge|charger|monitor [count]]\n");
      return 1;
    }

  return 0;
}
