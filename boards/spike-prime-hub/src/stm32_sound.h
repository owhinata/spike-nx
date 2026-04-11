/****************************************************************************
 * boards/spike-prime-hub/src/stm32_sound.h
 *
 * Private shared state for the SPIKE Prime Hub sound subsystem
 * (stm32_sound.c / stm32_tone.c / stm32_pcm.c).
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_SRC_STM32_SOUND_H
#define __BOARDS_SPIKE_PRIME_HUB_SRC_STM32_SOUND_H

#include <nuttx/config.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

enum sound_mode_e
{
  SOUND_MODE_IDLE = 0,
  SOUND_MODE_TONE,
  SOUND_MODE_PCM,
};

struct sound_state_s
{
  mutex_t              lock;        /* board-wide HW mutex */
  int                  open_count;  /* /dev/tone0 + /dev/pcm0 combined */
  FAR struct file     *owner;       /* current playback owner, NULL=idle */
  enum sound_mode_e    mode;        /* IDLE/TONE/PCM */
  uint8_t              volume;      /* 0..100 */
  atomic_bool          stop_flag;   /* tone_write interrupt request */
};

extern struct sound_state_s g_sound;

/* Low-level PCM API (caller must hold g_sound.lock). */

int  stm32_sound_play_pcm(FAR const uint16_t *data,
                          uint32_t length, uint32_t sample_rate);
int  stm32_sound_stop_pcm(void);
void stm32_sound_set_volume(uint8_t level);
uint8_t stm32_sound_get_volume(void);

#endif /* __BOARDS_SPIKE_PRIME_HUB_SRC_STM32_SOUND_H */
