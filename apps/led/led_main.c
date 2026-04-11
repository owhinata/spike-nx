/****************************************************************************
 * apps/led/led_main.c
 *
 * LED test utility for SPIKE Prime Hub TLC5955 driver.
 *
 * Usage:
 *   led green     - Status LED green (default boot state)
 *   led status    - Cycle status LED: R -> G -> B -> white -> off
 *   led battery   - Cycle battery LED: R -> G -> B -> white -> off
 *   led bluetooth - Cycle bluetooth LED: R -> G -> B -> white -> off
 *   led rainbow   - Rainbow animation on status LED
 *   led blink     - Blink status LED green
 *   led breathe   - Breathing effect on status LED
 *   led matrix    - 5x5 matrix patterns
 *   led all       - Run all tests
 *   led off       - All LEDs off
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "spike_prime_hub.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_DUTY  0xffff

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* 5x5 matrix channel map (row-major order) */

static const uint8_t g_matrix[5][5] =
{
  { 38, 36, 41, 46, 33 },  /* Row 0 */
  { 37, 28, 39, 47, 21 },  /* Row 1 */
  { 24, 29, 31, 45, 23 },  /* Row 2 */
  { 26, 27, 32, 34, 22 },  /* Row 3 */
  { 25, 40, 30, 35,  9 },  /* Row 4 */
};

/* 5x5 font for digits 0-9 (each row is 5 bits, MSB = col 0) */

static const uint8_t g_font[10][5] =
{
  { 0x0e, 0x11, 0x11, 0x11, 0x0e },  /* 0 */
  { 0x04, 0x0c, 0x04, 0x04, 0x0e },  /* 1 */
  { 0x0e, 0x01, 0x0e, 0x10, 0x1f },  /* 2 */
  { 0x0e, 0x01, 0x06, 0x01, 0x0e },  /* 3 */
  { 0x11, 0x11, 0x1f, 0x01, 0x01 },  /* 4 */
  { 0x1f, 0x10, 0x0e, 0x01, 0x1e },  /* 5 */
  { 0x0e, 0x10, 0x1e, 0x11, 0x0e },  /* 6 */
  { 0x1f, 0x01, 0x02, 0x04, 0x04 },  /* 7 */
  { 0x0e, 0x11, 0x0e, 0x11, 0x0e },  /* 8 */
  { 0x0e, 0x11, 0x0f, 0x01, 0x0e },  /* 9 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void set_status_rgb(uint16_t r, uint16_t g, uint16_t b)
{
  tlc5955_set_duty(TLC5955_CH_STATUS_TOP_R, r);
  tlc5955_set_duty(TLC5955_CH_STATUS_TOP_G, g);
  tlc5955_set_duty(TLC5955_CH_STATUS_TOP_B, b);
  tlc5955_set_duty(TLC5955_CH_STATUS_BTM_R, r);
  tlc5955_set_duty(TLC5955_CH_STATUS_BTM_G, g);
  tlc5955_set_duty(TLC5955_CH_STATUS_BTM_B, b);
}

static void set_battery_rgb(uint16_t r, uint16_t g, uint16_t b)
{
  tlc5955_set_duty(TLC5955_CH_BATTERY_R, r);
  tlc5955_set_duty(TLC5955_CH_BATTERY_G, g);
  tlc5955_set_duty(TLC5955_CH_BATTERY_B, b);
}

static void set_bt_rgb(uint16_t r, uint16_t g, uint16_t b)
{
  tlc5955_set_duty(TLC5955_CH_BT_R, r);
  tlc5955_set_duty(TLC5955_CH_BT_G, g);
  tlc5955_set_duty(TLC5955_CH_BT_B, b);
}

static void all_off(void)
{
  int i;

  for (i = 0; i < TLC5955_NUM_CHANNELS; i++)
    {
      tlc5955_set_duty(i, 0);
    }
}

static void matrix_clear(void)
{
  int r;
  int c;

  for (r = 0; r < 5; r++)
    {
      for (c = 0; c < 5; c++)
        {
          tlc5955_set_duty(g_matrix[r][c], 0);
        }
    }
}

static void matrix_show_digit(int digit)
{
  int r;
  int c;

  for (r = 0; r < 5; r++)
    {
      for (c = 0; c < 5; c++)
        {
          uint16_t val = (g_font[digit][r] & (0x10 >> c)) ? MAX_DUTY : 0;
          tlc5955_set_duty(g_matrix[r][c], val);
        }
    }
}

/* HSV to RGB conversion.
 * hue: 0-359, sat/val: 0-65535.
 * Output: r/g/b 0-65535.
 */

static void hsv_to_rgb(uint16_t hue, uint16_t sat, uint16_t val,
                       uint16_t *r, uint16_t *g, uint16_t *b)
{
  uint32_t h = hue % 360;
  uint32_t s = sat;
  uint32_t v = val;
  uint32_t c = (v * s) >> 16;
  uint32_t x;
  uint32_t m = v - c;
  uint32_t region = h / 60;
  uint32_t frac = h % 60;

  /* x = c * (1 - |frac/60*2 - 1|) */

  if (frac <= 30)
    {
      x = (c * frac * 2) / 60;
    }
  else
    {
      x = (c * (60 - frac) * 2) / 60;
    }

  switch (region)
    {
      case 0:  *r = c + m; *g = x + m; *b = m;     break;
      case 1:  *r = x + m; *g = c + m; *b = m;     break;
      case 2:  *r = m;     *g = c + m; *b = x + m; break;
      case 3:  *r = m;     *g = x + m; *b = c + m; break;
      case 4:  *r = x + m; *g = m;     *b = c + m; break;
      default: *r = c + m; *g = m;     *b = x + m; break;
    }
}

/****************************************************************************
 * Test Functions
 ****************************************************************************/

static void test_rgb_cycle(const char *name,
                           void (*set_rgb)(uint16_t, uint16_t, uint16_t))
{
  printf("%s: red\n", name);
  set_rgb(MAX_DUTY, 0, 0);
  sleep(1);

  printf("%s: green\n", name);
  set_rgb(0, MAX_DUTY, 0);
  sleep(1);

  printf("%s: blue\n", name);
  set_rgb(0, 0, MAX_DUTY);
  sleep(1);

  printf("%s: white\n", name);
  set_rgb(MAX_DUTY, MAX_DUTY, MAX_DUTY);
  sleep(1);

  printf("%s: off\n", name);
  set_rgb(0, 0, 0);
}

static void test_rainbow(void)
{
  int cycle;
  int hue;
  uint16_t r;
  uint16_t g;
  uint16_t b;

  printf("Rainbow: 3 cycles\n");

  for (cycle = 0; cycle < 3; cycle++)
    {
      for (hue = 0; hue < 360; hue += 5)
        {
          hsv_to_rgb(hue, MAX_DUTY, MAX_DUTY, &r, &g, &b);
          set_status_rgb(r, g, b);
          usleep(50000);
        }
    }

  set_status_rgb(0, 0, 0);
}

static void test_blink(void)
{
  int i;

  printf("Blink: green x5\n");

  for (i = 0; i < 5; i++)
    {
      set_status_rgb(0, MAX_DUTY, 0);
      usleep(500000);
      set_status_rgb(0, 0, 0);
      usleep(500000);
    }
}

static void test_breathe(void)
{
  int cycle;
  int step;
  uint16_t val;

  printf("Breathe: 2 cycles\n");

  for (cycle = 0; cycle < 2; cycle++)
    {
      /* Fade in */

      for (step = 0; step <= 255; step++)
        {
          val = (uint16_t)((uint32_t)step * MAX_DUTY / 255);
          set_status_rgb(0, val, 0);
          usleep(8000);
        }

      /* Fade out */

      for (step = 255; step >= 0; step--)
        {
          val = (uint16_t)((uint32_t)step * MAX_DUTY / 255);
          set_status_rgb(0, val, 0);
          usleep(8000);
        }
    }

  set_status_rgb(0, 0, 0);
}

static void test_matrix(void)
{
  int r;
  int c;
  int d;

  /* All on */

  printf("Matrix: all on\n");
  for (r = 0; r < 5; r++)
    {
      for (c = 0; c < 5; c++)
        {
          tlc5955_set_duty(g_matrix[r][c], MAX_DUTY);
        }
    }

  sleep(1);

  /* Scan */

  printf("Matrix: scan\n");
  matrix_clear();
  for (r = 0; r < 5; r++)
    {
      for (c = 0; c < 5; c++)
        {
          tlc5955_set_duty(g_matrix[r][c], MAX_DUTY);
          usleep(100000);
          tlc5955_set_duty(g_matrix[r][c], 0);
        }
    }

  /* Digits 0-9 */

  printf("Matrix: digits 0-9\n");
  for (d = 0; d <= 9; d++)
    {
      matrix_show_digit(d);
      usleep(500000);
    }

  matrix_clear();
}

static void test_green(void)
{
  set_status_rgb(0, MAX_DUTY, 0);
  printf("Status LED: green\n");
}

static void test_smoke(void)
{
  int r;
  int c;

  /* Light each LED group briefly in turn, then clear everything.
   * Used by the automated smoke test: proves each output channel is
   * reachable and that NSH returns control promptly (~0.5 s total).
   */

  printf("smoke: status\n");
  set_status_rgb(0, MAX_DUTY, 0);
  usleep(100000);
  set_status_rgb(0, 0, 0);

  printf("smoke: battery\n");
  set_battery_rgb(0, MAX_DUTY, 0);
  usleep(100000);
  set_battery_rgb(0, 0, 0);

  printf("smoke: bluetooth\n");
  set_bt_rgb(0, MAX_DUTY, 0);
  usleep(100000);
  set_bt_rgb(0, 0, 0);

  printf("smoke: matrix\n");
  for (r = 0; r < 5; r++)
    {
      for (c = 0; c < 5; c++)
        {
          tlc5955_set_duty(g_matrix[r][c], MAX_DUTY);
        }
    }
  usleep(100000);
  matrix_clear();

  printf("smoke: done\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int led_main(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      printf("Usage: led <command>\n");
      printf("  green     - Status LED green (boot default)\n");
      printf("  status    - Cycle status LED colors\n");
      printf("  battery   - Cycle battery LED colors\n");
      printf("  bluetooth - Cycle bluetooth LED colors\n");
      printf("  rainbow   - Rainbow animation\n");
      printf("  blink     - Blink green\n");
      printf("  breathe   - Breathing effect\n");
      printf("  matrix    - 5x5 matrix patterns\n");
      printf("  smoke     - Quick smoke test (each LED group once)\n");
      printf("  all       - Run all tests\n");
      printf("  off       - All LEDs off\n");
      return 1;
    }

  if (strcmp(argv[1], "green") == 0)
    {
      test_green();
    }
  else if (strcmp(argv[1], "status") == 0)
    {
      test_rgb_cycle("Status", set_status_rgb);
    }
  else if (strcmp(argv[1], "battery") == 0)
    {
      test_rgb_cycle("Battery", set_battery_rgb);
    }
  else if (strcmp(argv[1], "bluetooth") == 0)
    {
      test_rgb_cycle("Bluetooth", set_bt_rgb);
    }
  else if (strcmp(argv[1], "rainbow") == 0)
    {
      test_rainbow();
    }
  else if (strcmp(argv[1], "blink") == 0)
    {
      test_blink();
    }
  else if (strcmp(argv[1], "breathe") == 0)
    {
      test_breathe();
    }
  else if (strcmp(argv[1], "matrix") == 0)
    {
      test_matrix();
    }
  else if (strcmp(argv[1], "smoke") == 0)
    {
      test_smoke();
    }
  else if (strcmp(argv[1], "all") == 0)
    {
      test_rgb_cycle("Status", set_status_rgb);
      test_rgb_cycle("Battery", set_battery_rgb);
      test_rgb_cycle("Bluetooth", set_bt_rgb);
      test_rainbow();
      test_blink();
      test_breathe();
      test_matrix();
      printf("All tests done. Restoring green.\n");
      test_green();
    }
  else if (strcmp(argv[1], "off") == 0)
    {
      all_off();
      printf("All LEDs off\n");
    }
  else
    {
      printf("Unknown command: %s\n", argv[1]);
      return 1;
    }

  return 0;
}
