/****************************************************************************
 * apps/crashtest/crashtest_main.c
 *
 * Trigger various crash types to verify panic syslog dump output.
 *
 * Usage:
 *   crashtest assert    - Trigger ASSERT failure
 *   crashtest null      - Null pointer dereference
 *   crashtest divzero   - Division by zero
 *   crashtest stackoverflow - Stack overflow
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <assert.h>

static void crash_assert(void)
{
  printf("Triggering ASSERT...\n");
  ASSERT(false);
}

static volatile int g_zero = 0;

static void crash_null(void)
{
  /* Address 0xE0100000+ is reserved on Cortex-M and always faults.
   * Address 0 is valid (Flash) when MPU is disabled, so we use this instead.
   */

  volatile int *p = (volatile int *)0xEFFFFFFC;

  printf("Triggering bus fault (invalid memory access)...\n");
  *p = 42;
}

static void crash_divzero(void)
{
  volatile int x;

  printf("Triggering division by zero...\n");
  x = 1 / g_zero;
  (void)x;
}

static void crash_stackoverflow(void)
{
  volatile char buf[1024];

  buf[0] = 1;
  printf("Stack overflow depth...\n");
  crash_stackoverflow();
  (void)buf[0];
}

int crashtest_main(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      printf("Usage: crashtest <type>\n");
      printf("  assert         - ASSERT failure\n");
      printf("  null           - Null pointer dereference\n");
      printf("  divzero        - Division by zero\n");
      printf("  stackoverflow  - Stack overflow\n");
      return 1;
    }

  if (strcmp(argv[1], "assert") == 0)
    {
      crash_assert();
    }
  else if (strcmp(argv[1], "null") == 0)
    {
      crash_null();
    }
  else if (strcmp(argv[1], "divzero") == 0)
    {
      crash_divzero();
    }
  else if (strcmp(argv[1], "stackoverflow") == 0)
    {
      crash_stackoverflow();
    }
  else
    {
      printf("Unknown crash type: %s\n", argv[1]);
      return 1;
    }

  return 0;
}
