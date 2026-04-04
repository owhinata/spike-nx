/****************************************************************************
 * boards/spike-prime-hub/src/stm32_panic_syslog.c
 *
 * Syslog channel that outputs to stdout (USB CDC ACM) during panic/crash.
 * Silent during normal operation to avoid console noise.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/syslog/syslog.h>

static int panic_putc(FAR syslog_channel_t *channel, int ch)
{
  UNUSED(channel);

  if (g_nx_initstate == OSINIT_PANIC)
    {
      char c = (char)ch;
      write(1, &c, 1);
    }

  return ch;
}

static ssize_t panic_write(FAR syslog_channel_t *channel,
                           FAR const char *buf, size_t len)
{
  UNUSED(channel);

  if (g_nx_initstate == OSINIT_PANIC)
    {
      write(1, buf, len);
    }

  return len;
}

static int panic_flush(FAR syslog_channel_t *channel)
{
  UNUSED(channel);

  if (g_nx_initstate == OSINIT_PANIC)
    {
      fsync(1);
    }

  return OK;
}

static const struct syslog_channel_ops_s g_panic_channel_ops =
{
  .sc_putc        = panic_putc,
  .sc_force       = panic_putc,
  .sc_flush       = panic_flush,
  .sc_write       = panic_write,
  .sc_write_force = panic_write,
};

static syslog_channel_t g_panic_channel =
{
  .sc_ops = &g_panic_channel_ops,
};

void panic_syslog_initialize(void)
{
  syslog_channel_register(&g_panic_channel);
}
