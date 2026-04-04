/****************************************************************************
 * boards/spike-prime-hub/src/stm32_panic_syslog.c
 *
 * Syslog channel that outputs to serial console only during panic/crash.
 * Silent during normal operation to avoid console noise.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/syslog/syslog.h>

static int panic_putc(FAR syslog_channel_t *channel, int ch)
{
  UNUSED(channel);

  if (g_nx_initstate == OSINIT_PANIC)
    {
      up_putc(ch);
    }

  return ch;
}

static ssize_t panic_write(FAR syslog_channel_t *channel,
                           FAR const char *buf, size_t len)
{
  size_t i;

  UNUSED(channel);

  if (g_nx_initstate == OSINIT_PANIC)
    {
      for (i = 0; i < len; i++)
        {
          up_putc(buf[i]);
        }
    }

  return len;
}

static int panic_flush(FAR syslog_channel_t *channel)
{
  UNUSED(channel);
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
