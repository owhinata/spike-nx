/****************************************************************************
 * boards/spike-prime-hub/src/stm32_tone.c
 *
 * Tune-string char device /dev/tone0 for SPIKE Prime Hub.
 *
 * write() accepts pybricks-style note strings such as
 *   "C4/4 D#5/8. R/4 G4/4_"
 * where each token is:
 *   <note><accidental?><octave>/<fraction>[<dot>][<legato>]
 * or "R/<fraction>" for rests.  Parsing runs entirely in-kernel; each note
 * is played by (re)configuring DAC1/DMA1/TIM6 via stm32_sound_play_pcm and
 * sleeping with nxsig_usleep in 20 ms slices so that TONEIOC_STOP or a
 * parallel write from /dev/pcm0 can interrupt playback promptly.
 *
 * The tempo is fixed at 120 BPM (one whole note = 2 seconds) and can be
 * overridden by leading directives: "T<bpm>" (e.g. "T240 C4/4").
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <ctype.h>
#include <debug.h>
#include <errno.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>

#include <arch/board/board_sound.h>

#include "spike_prime_hub.h"
#include "stm32_sound.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NOTE_BUF_SAMPLES   256           /* max samples per period */
#define TARGET_SAMPLE_RATE 50000          /* target Hz, N adapts around this */
#define MIN_SAMPLE_RATE    1000
#define MAX_SAMPLE_RATE    100000
#define DEFAULT_TEMPO_BPM  120
#define SLICE_USEC         20000          /* 20 ms slice between stop checks */
#define MIN_MIDI           24             /* C1 */
#define MAX_MIDI           107            /* B7 */

/* volume curve: pybricks-style exponential from 0 to 0x7fff (half-scale
 * amplitude since samples are stored as uint16_t centered on 0x8000).
 */

static const uint16_t g_volume_curve[11] =
{
  0,    943,   2130,   3624,   5505,   7873,
  10851,  14607,  19331,  25280,  32767
};

/* midi note -> frequency in milli-Hz (Hz * 1000), 128 entries */

static const uint32_t g_midi_mhz[128] =
{
  8176,   8662,   9177,   9723,   10301,  10913,
  11562,  12250,  12978,  13750,  14568,  15434,
  16352,  17324,  18354,  19445,  20602,  21827,
  23125,  24500,  25957,  27500,  29135,  30868,
  32703,  34648,  36708,  38891,  41203,  43654,
  46249,  48999,  51913,  55000,  58270,  61735,
  65406,  69296,  73416,  77782,  82407,  87307,
  92499,  97999,  103826, 110000, 116541, 123471,
  130813, 138591, 146832, 155563, 164814, 174614,
  184997, 195998, 207652, 220000, 233082, 246942,
  261626, 277183, 293665, 311127, 329628, 349228,
  369994, 391995, 415305, 440000, 466164, 493883,
  523251, 554365, 587330, 622254, 659255, 698456,
  739989, 783991, 830609, 880000, 932328, 987767,
  1046502, 1108731, 1174659, 1244508, 1318510, 1396913,
  1479978, 1567982, 1661219, 1760000, 1864655, 1975533,
  2093005, 2217461, 2349318, 2489016, 2637020, 2793826,
  2959955, 3135963, 3322438, 3520000, 3729310, 3951066,
  4186009, 4434922, 4698636, 4978032, 5274041, 5587652,
  5919911, 6271927, 6644875, 7040000, 7458620, 7902133,
  8372018, 8869844, 9397273, 9956063, 10548082, 11175303,
  11839822, 12543854,
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct parsed_note_s
{
  bool     is_rest;
  bool     legato;
  int      midi;        /* valid only when !is_rest */
  uint32_t duration_us;
  uint32_t release_us;
};

struct tune_parser_s
{
  const char *cur;
  const char *end;
  int         tempo_bpm;
  int         default_octave;
  int         default_fraction;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DMA source buffer for tone playback.  One period of the current note. */

static uint16_t g_tone_buf[NOTE_BUF_SAMPLES];

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static uint16_t tone_amplitude_from_volume(uint8_t volume)
{
  if (volume > 100)
    {
      volume = 100;
    }

  uint32_t idx   = volume / 10;        /* 0..10 */
  uint32_t rem   = volume - idx * 10;  /* 0..9  */
  uint32_t lo    = g_volume_curve[idx];
  uint32_t hi    = (idx < 10) ? g_volume_curve[idx + 1] : g_volume_curve[10];
  uint32_t att   = lo + ((hi - lo) * rem) / 10;
  return (uint16_t)att;
}

/* Build one square-wave period into g_tone_buf[0..n-1].  Returns the sample
 * rate needed to play that period at freq_mhz (milli-Hz).
 */

static uint32_t tone_build_square(uint32_t freq_mhz, uint8_t volume,
                                  uint32_t *out_len)
{
  /* Choose an even sample count so that sample_rate stays in range.  Target
   * sample rate TARGET_SAMPLE_RATE (~50 kHz).  N = round(TARGET/freq).
   */

  uint32_t freq_hz = (freq_mhz + 500) / 1000;
  if (freq_hz == 0)
    {
      freq_hz = 1;
    }

  uint32_t n = TARGET_SAMPLE_RATE / freq_hz;
  if (n < 4)
    {
      n = 4;
    }
  if (n > NOTE_BUF_SAMPLES)
    {
      n = NOTE_BUF_SAMPLES;
    }

  n &= ~1U;  /* force even */

  /* Sample rate matching this period length for the requested frequency. */

  uint64_t sr = ((uint64_t)freq_mhz * n + 500) / 1000;
  if (sr < MIN_SAMPLE_RATE)
    {
      sr = MIN_SAMPLE_RATE;
    }
  if (sr > MAX_SAMPLE_RATE)
    {
      sr = MAX_SAMPLE_RATE;
    }

  uint16_t att  = tone_amplitude_from_volume(volume);
  uint16_t low  = 0x8000 - att;
  uint16_t high = 0x8000 + att;
  uint32_t half = n / 2;

  for (uint32_t i = 0; i < half; i++)
    {
      g_tone_buf[i] = low;
    }

  for (uint32_t i = half; i < n; i++)
    {
      g_tone_buf[i] = high;
    }

  *out_len = n;
  return (uint32_t)sr;
}

/* Fill a silent (center-value) buffer for rests so that the DAC does not
 * latch a high/low level between notes.
 */

static void tone_build_silence(uint32_t *out_len, uint32_t *out_sr)
{
  for (uint32_t i = 0; i < 16; i++)
    {
      g_tone_buf[i] = 0x8000;
    }

  *out_len = 16;
  *out_sr  = 16000;  /* arbitrary, silent buffer */
}

/****************************************************************************
 * Private: tune parser
 ****************************************************************************/

static void parser_skip_ws(struct tune_parser_s *p)
{
  while (p->cur < p->end &&
         (*p->cur == ' ' || *p->cur == '\t' || *p->cur == ',' ||
          *p->cur == '\r' || *p->cur == '\n'))
    {
      p->cur++;
    }
}

static int parser_read_int(struct tune_parser_s *p, int *out)
{
  if (p->cur >= p->end || !isdigit((unsigned char)*p->cur))
    {
      return -EINVAL;
    }

  int v = 0;
  while (p->cur < p->end && isdigit((unsigned char)*p->cur))
    {
      v = v * 10 + (*p->cur - '0');
      if (v > 100000)
        {
          return -EINVAL;
        }

      p->cur++;
    }

  *out = v;
  return OK;
}

static void parser_init(struct tune_parser_s *p, const char *buf, size_t len)
{
  p->cur              = buf;
  p->end              = buf + len;
  p->tempo_bpm        = DEFAULT_TEMPO_BPM;
  p->default_octave   = 4;
  p->default_fraction = 4;

  /* Leading directives: T<bpm>, O<octave>, L<fraction> (repeatable). */

  for (;;)
    {
      parser_skip_ws(p);
      if (p->cur >= p->end)
        {
          return;
        }

      char c = *p->cur;
      if (c == 'T' || c == 't')
        {
          p->cur++;
          int v;
          if (parser_read_int(p, &v) == OK && v >= 10 && v <= 400)
            {
              p->tempo_bpm = v;
            }
        }
      else if (c == 'O' || c == 'o')
        {
          p->cur++;
          int v;
          if (parser_read_int(p, &v) == OK && v >= 2 && v <= 8)
            {
              p->default_octave = v;
            }
        }
      else if (c == 'L' || c == 'l')
        {
          p->cur++;
          int v;
          if (parser_read_int(p, &v) == OK &&
              (v == 1 || v == 2 || v == 4 || v == 8 || v == 16 || v == 32))
            {
              p->default_fraction = v;
            }
        }
      else
        {
          return;
        }
    }
}

/* Convert note letter to semitone offset from C (C=0, C#=1, ..., B=11).
 * Returns -1 if not a note letter.
 */

static int parser_note_offset(char c)
{
  switch (c)
    {
      case 'C': case 'c': return 0;
      case 'D': case 'd': return 2;
      case 'E': case 'e': return 4;
      case 'F': case 'f': return 5;
      case 'G': case 'g': return 7;
      case 'A': case 'a': return 9;
      case 'B': case 'b': return 11;
      default:            return -1;
    }
}

/* Parse the next token.  Returns OK and fills note, or -ENODATA at end,
 * or -EINVAL on a malformed token (the caller may skip forward).
 */

static int parser_next(struct tune_parser_s *p, struct parsed_note_s *note)
{
  parser_skip_ws(p);
  if (p->cur >= p->end)
    {
      return -ENODATA;
    }

  note->is_rest = false;
  note->legato  = false;

  char c = *p->cur;

  if (c == 'R' || c == 'r')
    {
      note->is_rest = true;
      p->cur++;
    }
  else
    {
      int off = parser_note_offset(c);
      if (off < 0)
        {
          p->cur++;
          return -EINVAL;
        }

      p->cur++;

      /* Optional accidental. */

      if (p->cur < p->end && *p->cur == '#')
        {
          off++;
          p->cur++;
        }
      else if (p->cur < p->end && *p->cur == 'b')
        {
          off--;
          p->cur++;
        }

      /* Optional octave (digit); otherwise default. */

      int octave = p->default_octave;
      if (p->cur < p->end && isdigit((unsigned char)*p->cur))
        {
          octave = *p->cur - '0';
          p->cur++;
        }

      int midi = (octave + 1) * 12 + off;
      if (midi < MIN_MIDI || midi > MAX_MIDI)
        {
          return -EINVAL;
        }

      note->midi = midi;
    }

  /* Optional /<fraction>. */

  int fraction = p->default_fraction;
  if (p->cur < p->end && *p->cur == '/')
    {
      p->cur++;
      int v;
      if (parser_read_int(p, &v) == OK &&
          (v == 1 || v == 2 || v == 4 || v == 8 || v == 16 || v == 32))
        {
          fraction = v;
        }
      else
        {
          return -EINVAL;
        }
    }

  /* Optional dot / legato suffixes. */

  bool dotted = false;
  while (p->cur < p->end)
    {
      if (*p->cur == '.')
        {
          dotted = true;
          p->cur++;
        }
      else if (*p->cur == '_')
        {
          note->legato = true;
          p->cur++;
        }
      else
        {
          break;
        }
    }

  /* Note length in microseconds.  whole_note_us = 4 * 60 * 1e6 / bpm. */

  uint64_t whole_us = (uint64_t)240000000 / (uint32_t)p->tempo_bpm;
  uint64_t dur_us   = whole_us / (uint32_t)fraction;
  if (dotted)
    {
      dur_us += dur_us / 2;
    }

  note->duration_us = (uint32_t)dur_us;
  note->release_us  = note->legato ? 0u : (uint32_t)(dur_us / 8);
  note->duration_us -= note->release_us;
  return OK;
}

/****************************************************************************
 * Private: char device entry points
 ****************************************************************************/

static int tone_open(FAR struct file *filep)
{
  nxmutex_lock(&g_sound.lock);
  g_sound.open_count++;
  nxmutex_unlock(&g_sound.lock);
  return OK;
}

static int tone_close(FAR struct file *filep)
{
  nxmutex_lock(&g_sound.lock);
  g_sound.open_count--;

  if (g_sound.owner == filep)
    {
      atomic_store_explicit(&g_sound.stop_flag, true, memory_order_relaxed);
      stm32_sound_stop_pcm();
      g_sound.owner = NULL;
      g_sound.mode  = SOUND_MODE_IDLE;
    }
  else if (g_sound.open_count == 0)
    {
      stm32_sound_stop_pcm();
      g_sound.mode = SOUND_MODE_IDLE;
    }

  nxmutex_unlock(&g_sound.lock);
  return OK;
}

/* Sleep in 20 ms slices while polling stop_flag.  Returns -EINTR on stop. */

static int tone_sleep_with_stop(uint32_t remaining_us)
{
  while (remaining_us > 0)
    {
      if (atomic_load_explicit(&g_sound.stop_flag, memory_order_relaxed))
        {
          return -EINTR;
        }

      uint32_t slice = (remaining_us > SLICE_USEC) ? SLICE_USEC : remaining_us;
      int ret = nxsig_usleep(slice);
      if (ret < 0)
        {
          return -EINTR;
        }

      remaining_us -= slice;
    }

  return OK;
}

static ssize_t tone_write(FAR struct file *filep,
                          FAR const char *ubuf, size_t nbytes)
{
  if (ubuf == NULL || nbytes == 0)
    {
      return 0;
    }

  struct tune_parser_s parser;
  struct parsed_note_s note;
  ssize_t              ret = (ssize_t)nbytes;

  parser_init(&parser, ubuf, nbytes);

  /* Claim the hardware and clear any pending stop request. */

  nxmutex_lock(&g_sound.lock);
  g_sound.owner = filep;
  g_sound.mode  = SOUND_MODE_TONE;
  atomic_store_explicit(&g_sound.stop_flag, false, memory_order_relaxed);
  nxmutex_unlock(&g_sound.lock);

  for (;;)
    {
      int perr = parser_next(&parser, &note);
      if (perr == -ENODATA)
        {
          break;
        }
      else if (perr == -EINVAL)
        {
          /* Skip malformed token and continue. */

          continue;
        }

      if (atomic_load_explicit(&g_sound.stop_flag, memory_order_relaxed))
        {
          ret = -EINTR;
          goto out;
        }

      /* Build and start the waveform for this note under the lock. */

      nxmutex_lock(&g_sound.lock);
      if (g_sound.owner != filep)
        {
          nxmutex_unlock(&g_sound.lock);
          ret = -EINTR;
          goto out_no_stop;
        }

      uint32_t length;
      uint32_t sample_rate;

      if (note.is_rest)
        {
          tone_build_silence(&length, &sample_rate);
          stm32_sound_stop_pcm();  /* mute instead of playing silence */
        }
      else
        {
          sample_rate = tone_build_square(g_midi_mhz[note.midi],
                                          g_sound.volume, &length);
          stm32_sound_play_pcm(g_tone_buf, length, sample_rate);
        }

      nxmutex_unlock(&g_sound.lock);

      /* Active portion of the note. */

      int s = tone_sleep_with_stop(note.duration_us);
      if (s < 0)
        {
          ret = s;
          goto out;
        }

      /* Release gap (silent) between notes unless legato. */

      if (note.release_us > 0)
        {
          nxmutex_lock(&g_sound.lock);
          if (g_sound.owner == filep)
            {
              stm32_sound_stop_pcm();
            }
          nxmutex_unlock(&g_sound.lock);

          s = tone_sleep_with_stop(note.release_us);
          if (s < 0)
            {
              ret = s;
              goto out;
            }
        }
    }

out:
  nxmutex_lock(&g_sound.lock);
  if (g_sound.owner == filep)
    {
      stm32_sound_stop_pcm();
      g_sound.owner = NULL;
      g_sound.mode  = SOUND_MODE_IDLE;
    }

  nxmutex_unlock(&g_sound.lock);

out_no_stop:
  return ret;
}

static int tone_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  switch (cmd)
    {
      case TONEIOC_VOLUME_SET:
        {
          nxmutex_lock(&g_sound.lock);
          stm32_sound_set_volume((uint8_t)arg);
          nxmutex_unlock(&g_sound.lock);
          return OK;
        }

      case TONEIOC_VOLUME_GET:
        {
          FAR int *out = (FAR int *)((uintptr_t)arg);
          if (out == NULL)
            {
              return -EINVAL;
            }

          *out = (int)stm32_sound_get_volume();
          return OK;
        }

      case TONEIOC_STOP:
        {
          nxmutex_lock(&g_sound.lock);
          atomic_store_explicit(&g_sound.stop_flag, true,
                                memory_order_relaxed);
          stm32_sound_stop_pcm();
          g_sound.owner = NULL;
          g_sound.mode  = SOUND_MODE_IDLE;
          nxmutex_unlock(&g_sound.lock);
          return OK;
        }

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Private Data (file operations)
 ****************************************************************************/

static const struct file_operations g_tone_fops =
{
  .open  = tone_open,
  .close = tone_close,
  .read  = NULL,
  .write = tone_write,
  .seek  = NULL,
  .ioctl = tone_ioctl,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_tone_register(void)
{
  int ret = register_driver("/dev/tone0", &g_tone_fops, 0666, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "tone: register_driver failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "tone: /dev/tone0 registered\n");
  return OK;
}
