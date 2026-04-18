/****************************************************************************
 * boards/spike-prime-hub/include/board_rgbled.h
 *
 * Public ABI for the SPIKE Prime Hub TLC5955 LED char device:
 *   /dev/rgbled0 -- 48-channel PWM LED controller (status / battery /
 *                   bluetooth RGB groups + 5x5 matrix + charger mode)
 *
 * The underlying chip is a 48-channel grayscale PWM driver; channel
 * layout is exposed as TLC5955_CH_* below so user-space utilities can
 * target individual LEDs without talking to the kernel directly.
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_RGBLED_H
#define __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_RGBLED_H

#include <nuttx/fs/ioctl.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Channel count of the TLC5955 shift register */

#define TLC5955_NUM_CHANNELS  48

/* Channel IDs (identical to the hardware wiring on SPIKE Prime Hub) */

#define TLC5955_CH_BATTERY_B       0
#define TLC5955_CH_BATTERY_G       1
#define TLC5955_CH_BATTERY_R       2
#define TLC5955_CH_STATUS_TOP_B    3
#define TLC5955_CH_STATUS_TOP_G    4
#define TLC5955_CH_STATUS_TOP_R    5
#define TLC5955_CH_STATUS_BTM_B    6
#define TLC5955_CH_STATUS_BTM_G    7
#define TLC5955_CH_STATUS_BTM_R    8
#define TLC5955_CH_CHARGER_MODE   14  /* active-low: duty=0 enables charging */
#define TLC5955_CH_BT_B           18
#define TLC5955_CH_BT_G           19
#define TLC5955_CH_BT_R           20

/* Board-local ioctl command space.  Uses the _BOARDIOC range; sound
 * commands occupy 0x40..0x4F, so rgbled takes 0x50..0x5F.
 */

#define RGBLEDIOC_SETDUTY  _BOARDIOC(0x50)  /* struct rgbled_duty_s *arg */
#define RGBLEDIOC_SETALL   _BOARDIOC(0x51)  /* uint16_t duty (via arg)  */
#define RGBLEDIOC_UPDATE   _BOARDIOC(0x52)  /* arg = 0 (force flush)    */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct rgbled_duty_s
{
  uint8_t  channel;  /* 0 .. TLC5955_NUM_CHANNELS - 1 */
  uint16_t value;    /* 0 (off) .. 0xffff (full brightness) */
};

#endif /* __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_RGBLED_H */
