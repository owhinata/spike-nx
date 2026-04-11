/****************************************************************************
 * apps/sound/sound_main.c
 *
 * Sound test utility for SPIKE Prime Hub.
 *
 * Usage:
 *   sound beep [freq_hz] [dur_ms]    Play a square wave via /dev/pcm0
 *   sound tone <freq> <dur_ms>       Alias for beep
 *   sound notes <tune>               Write tune string to /dev/tone0
 *   sound volume [0-100]             Get or set volume via TONEIOC_VOLUME_*
 *   sound off                        Stop playback via TONEIOC_STOP
 *   sound selftest                   Play 500 Hz for 200 ms (boot verify)
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <arch/board/board_sound.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TONE_DEV  "/dev/tone0"
#define PCM_DEV   "/dev/pcm0"

#define PCM_MAX_SAMPLES 256
#define TARGET_SR       50000
#define MIN_SR          1000
#define MAX_SR          100000
#define AMPLITUDE       0x3f00  /* ~half of 0x7fff, pleasant volume */

/****************************************************************************
 * Private: waveform construction
 ****************************************************************************/

struct pcm_blob_s
{
  struct pcm_write_hdr_s hdr;
  uint16_t               samples[PCM_MAX_SAMPLES];
};

/* Build one period of a square wave into blob and compute the matching
 * sample rate.  Returns blob total size in bytes.
 */

static size_t build_square(struct pcm_blob_s *blob, uint32_t freq_hz)
{
  if (freq_hz == 0)
    {
      freq_hz = 1;
    }

  uint32_t n = TARGET_SR / freq_hz;
  if (n < 4)
    {
      n = 4;
    }

  if (n > PCM_MAX_SAMPLES)
    {
      n = PCM_MAX_SAMPLES;
    }

  n &= ~1U;  /* even */

  uint32_t sr = freq_hz * n;
  if (sr < MIN_SR)
    {
      sr = MIN_SR;
    }

  if (sr > MAX_SR)
    {
      sr = MAX_SR;
    }

  uint16_t low  = 0x8000 - AMPLITUDE;
  uint16_t high = 0x8000 + AMPLITUDE;
  uint32_t half = n / 2;

  for (uint32_t i = 0; i < half; i++)
    {
      blob->samples[i] = low;
    }

  for (uint32_t i = half; i < n; i++)
    {
      blob->samples[i] = high;
    }

  blob->hdr.magic        = PCM_WRITE_MAGIC;
  blob->hdr.version      = PCM_WRITE_VERSION;
  blob->hdr.hdr_size     = sizeof(struct pcm_write_hdr_s);
  blob->hdr.flags        = 0;
  blob->hdr.sample_rate  = sr;
  blob->hdr.sample_count = n;

  return sizeof(struct pcm_write_hdr_s) + n * sizeof(uint16_t);
}

/****************************************************************************
 * Private: subcommand handlers
 ****************************************************************************/

static int cmd_beep(uint32_t freq_hz, uint32_t dur_ms)
{
  struct pcm_blob_s blob;
  size_t            total = build_square(&blob, freq_hz);

  int fd = open(PCM_DEV, O_WRONLY);
  if (fd < 0)
    {
      fprintf(stderr, "sound: open(%s) failed: %d\n", PCM_DEV, errno);
      return 1;
    }

  ssize_t w = write(fd, &blob, total);
  if (w < 0)
    {
      fprintf(stderr, "sound: write failed: %d\n", errno);
      close(fd);
      return 1;
    }

  usleep(dur_ms * 1000);

  ioctl(fd, TONEIOC_STOP, 0);
  close(fd);
  return 0;
}

static int cmd_notes(const char *tune)
{
  int fd = open(TONE_DEV, O_WRONLY);
  if (fd < 0)
    {
      fprintf(stderr, "sound: open(%s) failed: %d\n", TONE_DEV, errno);
      return 1;
    }

  size_t len = strlen(tune);
  ssize_t w  = write(fd, tune, len);
  if (w < 0)
    {
      fprintf(stderr, "sound: write failed: %d\n", errno);
      close(fd);
      return 1;
    }

  close(fd);
  return 0;
}

static int cmd_volume(int level)
{
  int fd = open(TONE_DEV, O_RDWR);
  if (fd < 0)
    {
      fprintf(stderr, "sound: open(%s) failed: %d\n", TONE_DEV, errno);
      return 1;
    }

  int ret;

  if (level < 0)
    {
      int cur = 0;
      ret = ioctl(fd, TONEIOC_VOLUME_GET, (unsigned long)(uintptr_t)&cur);
      if (ret < 0)
        {
          fprintf(stderr, "sound: VOLUME_GET failed: %d\n", errno);
          close(fd);
          return 1;
        }

      printf("volume: %d\n", cur);
    }
  else
    {
      if (level > 100)
        {
          level = 100;
        }

      ret = ioctl(fd, TONEIOC_VOLUME_SET, (unsigned long)level);
      if (ret < 0)
        {
          fprintf(stderr, "sound: VOLUME_SET failed: %d\n", errno);
          close(fd);
          return 1;
        }
    }

  close(fd);
  return 0;
}

static int cmd_off(void)
{
  int fd = open(TONE_DEV, O_RDWR);
  if (fd < 0)
    {
      fprintf(stderr, "sound: open(%s) failed: %d\n", TONE_DEV, errno);
      return 1;
    }

  ioctl(fd, TONEIOC_STOP, 0);
  close(fd);
  return 0;
}

static int cmd_selftest(void)
{
  printf("sound: selftest 500 Hz 200 ms via /dev/pcm0\n");
  return cmd_beep(500, 200);
}

/****************************************************************************
 * Usage
 ****************************************************************************/

static void usage(void)
{
  fprintf(stderr,
          "Usage:\n"
          "  sound beep [freq_hz] [dur_ms]   (default 500 Hz 200 ms)\n"
          "  sound tone <freq> <dur_ms>      alias for beep\n"
          "  sound notes <tune>              e.g. \"T120 C4/4 E4/4 G4/2\"\n"
          "  sound volume [0-100]            get or set volume\n"
          "  sound off                       stop playback\n"
          "  sound selftest                  play 500 Hz 200 ms\n");
}

/****************************************************************************
 * main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      usage();
      return 1;
    }

  const char *cmd = argv[1];

  if (strcmp(cmd, "beep") == 0 || strcmp(cmd, "tone") == 0)
    {
      uint32_t freq = 500;
      uint32_t dur  = 200;
      if (argc >= 3)
        {
          freq = (uint32_t)atoi(argv[2]);
        }

      if (argc >= 4)
        {
          dur = (uint32_t)atoi(argv[3]);
        }

      return cmd_beep(freq, dur);
    }

  if (strcmp(cmd, "notes") == 0)
    {
      if (argc < 3)
        {
          usage();
          return 1;
        }

      /* Join argv[2..argc-1] with spaces so NSH argument splitting does
       * not drop everything after the first whitespace when the tune is
       * passed without quotes (or when NSH strips quotes).
       */

      char buf[256];
      size_t pos = 0;

      for (int i = 2; i < argc && pos < sizeof(buf) - 1; i++)
        {
          size_t len = strlen(argv[i]);
          if (i > 2 && pos < sizeof(buf) - 1)
            {
              buf[pos++] = ' ';
            }

          if (pos + len >= sizeof(buf))
            {
              len = sizeof(buf) - 1 - pos;
            }

          memcpy(buf + pos, argv[i], len);
          pos += len;
        }

      buf[pos] = '\0';
      return cmd_notes(buf);
    }

  if (strcmp(cmd, "volume") == 0)
    {
      int level = -1;
      if (argc >= 3)
        {
          level = atoi(argv[2]);
        }

      return cmd_volume(level);
    }

  if (strcmp(cmd, "off") == 0)
    {
      return cmd_off();
    }

  if (strcmp(cmd, "selftest") == 0)
    {
      return cmd_selftest();
    }

  usage();
  return 1;
}
