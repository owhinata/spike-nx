/****************************************************************************
 * apps/crashme/crashme_main.c
 *
 * Intentionally crash the system in various ways for testing stack dump
 * analysis.
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>

static void func_c(void)
{
  /* Null pointer dereference */

  *(volatile int *)0 = 0xdead;
}

static void func_b(void)
{
  func_c();
}

static void func_a(void)
{
  func_b();
}

static void crash_stackoverflow(int depth)
{
  volatile char buf[256];
  memset((void *)buf, 0xaa, sizeof(buf));
  printf("depth=%d sp~%p\n", depth, (void *)buf);
  crash_stackoverflow(depth + 1);
}

int main(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      printf("Usage: crashme <mode>\n");
      printf("  null   - null pointer dereference\n");
      printf("  assert - ASSERT(false)\n");
      printf("  stack  - stack overflow\n");
      return 1;
    }

  if (strcmp(argv[1], "null") == 0)
    {
      printf("Crashing via null pointer dereference...\n");
      func_a();
    }
  else if (strcmp(argv[1], "assert") == 0)
    {
      printf("Crashing via ASSERT...\n");
      ASSERT(false);
    }
  else if (strcmp(argv[1], "stack") == 0)
    {
      printf("Crashing via stack overflow...\n");
      crash_stackoverflow(0);
    }
  else
    {
      printf("Unknown mode: %s\n", argv[1]);
      return 1;
    }

  return 0;
}
