/****************************************************************************
 * boards/spike-prime-hub/include/board_sound.h
 *
 * Public ABI for the SPIKE Prime Hub sound char devices:
 *   /dev/tone0  -- tune-string char device (pybricks "C4/4" syntax)
 *   /dev/pcm0   -- raw PCM single-call ABI (pbdrv_sound_start compatible)
 *
 * Both devices share the same ioctl command space (TONEIOC_*).
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_SOUND_H
#define __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_SOUND_H

#include <nuttx/fs/ioctl.h>
#include <stdint.h>

/* Board-local ioctl command space.  _BOARDIOCBASE is reserved by NuttX for
 * board-specific codes so we do not collide with AUDIOIOC_* or SNDIOC_*.
 */

#define TONEIOC_VOLUME_SET _BOARDIOC(0x40)  /* int arg, 0..100 */
#define TONEIOC_VOLUME_GET _BOARDIOC(0x41)  /* int *arg         */
#define TONEIOC_STOP       _BOARDIOC(0x42)  /* arg = 0          */

/* /dev/pcm0 single-call write header (ABI v1).
 *
 * write(fd, buf, nbytes) expects:
 *   - buf starts with a struct pcm_write_hdr_s
 *   - magic == PCM_WRITE_MAGIC
 *   - version <= PCM_WRITE_VERSION
 *   - hdr_size >= sizeof(struct pcm_write_hdr_s) and <= nbytes
 *   - sample_rate in [1000, 100000]
 *   - (nbytes - hdr_size) == sample_count * sizeof(uint16_t)
 *
 * Samples are 16-bit unsigned, centered on 0x8000 (left-aligned DAC).
 */

#define PCM_WRITE_MAGIC    0x304D4350u  /* 'P' 'C' 'M' '0' little-endian */
#define PCM_WRITE_VERSION  0x0001u

struct pcm_write_hdr_s
{
  uint32_t magic;        /* PCM_WRITE_MAGIC */
  uint16_t version;      /* ABI version, currently 1 */
  uint16_t hdr_size;     /* sizeof(struct) or larger for future extensions */
  uint32_t flags;        /* 0 only for now (reserved) */
  uint32_t sample_rate;  /* Hz, [1000, 100000] */
  uint32_t sample_count; /* number of uint16_t samples following the header */
};

#endif /* __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_SOUND_H */
