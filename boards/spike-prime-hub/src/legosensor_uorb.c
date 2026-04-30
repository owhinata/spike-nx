/****************************************************************************
 * boards/spike-prime-hub/src/legosensor_uorb.c
 *
 * LEGO Powered Up sensor uORB driver (Issue #45) — publishes telemetry
 * from any LUMP-capable device (Color, Ultrasonic, Force sensors and
 * BOOST/Technic/SPIKE motor encoders) to /dev/uorb/sensor_lego[0..5].
 *
 * Sits on top of the LUMP UART protocol engine (Issue #43): receives
 * `on_sync` / `on_data` / `on_error` callbacks per port and pushes
 * fixed-size `struct lump_sample_s` envelopes through the NuttX uORB
 * sensor framework.  Mode switching and writable-mode TX are handled
 * via a custom `control()` ioctl in Phase 2.
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version
 * 2.0 (the "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/nuttx.h>

#include <debug.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/sensors/sensor.h>

#include <arch/board/board_legosensor.h>
#include <arch/board/board_lump.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LEGOSENSOR_NO_PENDING        0xffu
#define LEGOSENSOR_PENDING_TIMEOUT_MS 500

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct legosensor_dev_s
{
  struct sensor_lowerhalf_s lower;
  mutex_t  lock;
  int      port;                /* 0..5 */
  bool     registered;          /* sensor_custom_register succeeded */
  bool     attached;            /* lump_attach succeeded */
  bool     synced;              /* SYNC complete, info_cache valid */

  /* Telemetry state (mutex-protected) */

  uint32_t seq;
  uint32_t generation;
  uint8_t  type_id;
  uint8_t  mode_id;
  uint8_t  pending_select_mode;
  clock_t  pending_select_deadline;

  /* Phase 2 fields (placeholder so future patch sets owner safely) */

  FAR struct file *claim_owner;

  /* Last known device info (snapshotted from on_sync). */

  struct lump_device_info_s info_cache;

  char     path[32];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int legosensor_open(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep);
static int legosensor_close(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep);
static int legosensor_activate(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep, bool enable);
static int legosensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep,
                                   FAR uint32_t *interval);
static int legosensor_control(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              int cmd, unsigned long arg);

static void legosensor_on_sync(int port,
                               FAR const struct lump_device_info_s *info,
                               FAR void *priv);
static void legosensor_on_data(int port, uint8_t mode,
                               FAR const uint8_t *data, size_t len,
                               FAR void *priv);
static void legosensor_on_error(int port, uint8_t mode,
                                FAR const uint8_t *data, size_t len,
                                FAR void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct legosensor_dev_s g_legosensor[LEGOSENSOR_NUM_PORTS];

static const struct sensor_ops_s g_legosensor_ops =
{
  .open         = legosensor_open,
  .close        = legosensor_close,
  .activate     = legosensor_activate,
  .set_interval = legosensor_set_interval,
  .control      = legosensor_control,
};

static const struct lump_callbacks_s g_legosensor_lump_cb =
{
  .on_sync  = legosensor_on_sync,
  .on_data  = legosensor_on_data,
  .on_error = legosensor_on_error,
  .priv     = NULL,             /* per-port priv set at attach */
};

/****************************************************************************
 * Private Functions — envelope helpers
 ****************************************************************************/

/* Build a sample envelope from current state and the given LUMP frame.
 * Caller MUST hold dev->lock; the envelope is returned by value so the
 * caller can release the lock before invoking push_event().
 */

static void legosensor_build_envelope(FAR struct legosensor_dev_s *dev,
                                      uint8_t mode,
                                      FAR const uint8_t *data, size_t len,
                                      FAR struct lump_sample_s *out)
{
  uint8_t  num_values = 0;
  uint8_t  data_type  = 0;
  size_t   copy_len   = len > LEGOSENSOR_MAX_DATA_BYTES ?
                        LEGOSENSOR_MAX_DATA_BYTES : len;

  if (dev->synced && mode < dev->info_cache.num_modes)
    {
      num_values = dev->info_cache.modes[mode].num_values;
      data_type  = dev->info_cache.modes[mode].data_type;
    }

  memset(out, 0, sizeof(*out));
  out->timestamp  = sensor_get_timestamp();
  out->seq        = dev->seq;
  out->generation = dev->generation;
  out->port       = (uint8_t)dev->port;
  out->type_id    = dev->type_id;
  out->mode_id    = mode;
  out->data_type  = data_type;
  out->num_values = num_values;
  out->len        = (uint8_t)copy_len;

  if (data != NULL && copy_len > 0)
    {
      memcpy(out->data.raw, data, copy_len);
    }
}

/****************************************************************************
 * Private Functions — LUMP callbacks (per-port kthread context)
 ****************************************************************************/

static void legosensor_on_sync(int port,
                               FAR const struct lump_device_info_s *info,
                               FAR void *priv)
{
  FAR struct legosensor_dev_s *dev = (FAR struct legosensor_dev_s *)priv;
  struct lump_sample_s sentinel;

  if (dev == NULL || info == NULL)
    {
      return;
    }

  nxmutex_lock(&dev->lock);

  /* Snapshot device info; clear any stale pending SELECT. */

  memcpy(&dev->info_cache, info, sizeof(dev->info_cache));
  dev->synced              = true;
  dev->type_id             = info->type_id;
  dev->mode_id             = info->current_mode;
  dev->pending_select_mode = LEGOSENSOR_NO_PENDING;
  dev->generation++;
  dev->seq++;

  /* Sync sentinel: type_id != 0, len == 0.  Pre-warms the upper-half
   * circbuf so the first real on_data does not pay the kmm allocation
   * cost on the LUMP kthread.
   */

  legosensor_build_envelope(dev, dev->mode_id, NULL, 0, &sentinel);

  nxmutex_unlock(&dev->lock);

  dev->lower.push_event(dev->lower.priv, &sentinel, sizeof(sentinel));
}

static void legosensor_on_data(int port, uint8_t mode,
                               FAR const uint8_t *data, size_t len,
                               FAR void *priv)
{
  FAR struct legosensor_dev_s *dev = (FAR struct legosensor_dev_s *)priv;
  struct lump_sample_s sample;
  clock_t now;
  bool need_hydrate;

  if (dev == NULL)
    {
      return;
    }

  /* Lazy info_cache hydration.  `lump_attach()` only fires `on_sync`
   * synchronously when the engine is *already* SYNCED at attach time;
   * for the common boot path (attach before any device is plugged in)
   * `on_sync` never fires, leaving num_values / data_type at 0.
   *
   * Refresh from the engine once when the cache looks empty, and
   * re-refresh whenever the engine's reported `type_id` changes
   * (covers hot-unplug + reconnect-with-different-device because
   * on_error is currently a no-op in the LUMP engine).
   */

  nxmutex_lock(&dev->lock);
  need_hydrate = !dev->synced || dev->info_cache.num_modes == 0;
  nxmutex_unlock(&dev->lock);

  if (need_hydrate)
    {
      struct lump_device_info_s info;

      if (lump_get_info(port, &info) == OK)
        {
          struct lump_sample_s sentinel;
          bool fire_sync_sentinel = false;

          nxmutex_lock(&dev->lock);
          if (info.type_id != dev->info_cache.type_id ||
              info.num_modes != dev->info_cache.num_modes)
            {
              memcpy(&dev->info_cache, &info, sizeof(dev->info_cache));
              dev->type_id = info.type_id;
              dev->mode_id = info.current_mode;
              dev->generation++;
              dev->seq++;
              dev->synced = true;
              fire_sync_sentinel = true;

              legosensor_build_envelope(dev, dev->mode_id, NULL, 0,
                                        &sentinel);
            }
          else if (!dev->synced)
            {
              dev->synced = true;
            }
          nxmutex_unlock(&dev->lock);

          if (fire_sync_sentinel)
            {
              dev->lower.push_event(dev->lower.priv, &sentinel,
                                    sizeof(sentinel));
            }
        }
    }

  nxmutex_lock(&dev->lock);

  /* Generation update on confirmed SELECT (Codex-recommended timing).
   * We bump only when the device actually starts reporting the mode
   * the user asked for; expired pending requests are silently dropped.
   */

  if (dev->pending_select_mode != LEGOSENSOR_NO_PENDING)
    {
      now = clock_systime_ticks();
      if ((int32_t)(now - dev->pending_select_deadline) >= 0)
        {
          dev->pending_select_mode = LEGOSENSOR_NO_PENDING;
        }
      else if (mode == dev->pending_select_mode)
        {
          dev->generation++;
          dev->pending_select_mode = LEGOSENSOR_NO_PENDING;
        }
    }

  dev->mode_id = mode;
  dev->seq++;

  legosensor_build_envelope(dev, mode, data, len, &sample);

  nxmutex_unlock(&dev->lock);

  dev->lower.push_event(dev->lower.priv, &sample, sizeof(sample));
}

static void legosensor_on_error(int port, uint8_t mode,
                                FAR const uint8_t *data, size_t len,
                                FAR void *priv)
{
  FAR struct legosensor_dev_s *dev = (FAR struct legosensor_dev_s *)priv;
  struct lump_sample_s sentinel;

  UNUSED(mode);
  UNUSED(data);
  UNUSED(len);

  if (dev == NULL)
    {
      return;
    }

  nxmutex_lock(&dev->lock);

  dev->synced              = false;
  dev->type_id             = 0;
  dev->mode_id             = 0;
  dev->pending_select_mode = LEGOSENSOR_NO_PENDING;
  dev->claim_owner         = NULL;          /* drop control on disconnect */
  dev->generation++;
  dev->seq++;
  memset(&dev->info_cache, 0, sizeof(dev->info_cache));

  /* Disconnect sentinel: type_id == 0, len == 0. */

  memset(&sentinel, 0, sizeof(sentinel));
  sentinel.timestamp  = sensor_get_timestamp();
  sentinel.seq        = dev->seq;
  sentinel.generation = dev->generation;
  sentinel.port       = (uint8_t)dev->port;
  /* type_id, mode_id, data_type, num_values, len already zero */

  nxmutex_unlock(&dev->lock);

  dev->lower.push_event(dev->lower.priv, &sentinel, sizeof(sentinel));
}

/****************************************************************************
 * Private Functions — sensor_ops_s
 ****************************************************************************/

static int legosensor_open(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep)
{
  UNUSED(lower);

  /* Reject write opens — the only legitimate writer is the LUMP kthread
   * via on_data().  Allowing O_WROK would let userspace inject fake
   * samples through sensor_write(), bypassing the LUMP transport.
   */

  if ((filep->f_oflags & O_WROK) != 0)
    {
      return -EACCES;
    }

  return OK;
}

static int legosensor_close(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep)
{
  FAR struct legosensor_dev_s *dev =
      container_of(lower, struct legosensor_dev_s, lower);

  /* Auto-release the CLAIM if the closing fd held it.  NuttX 12.13.0
   * sensor_close() invokes ops->close() per-fd, so this fires before
   * the filep pointer is reused for a new open.
   */

  nxmutex_lock(&dev->lock);
  if (dev->claim_owner == filep)
    {
      dev->claim_owner = NULL;
    }
  nxmutex_unlock(&dev->lock);

  return OK;
}

static int legosensor_activate(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep, bool enable)
{
  /* The LUMP engine is always running; activate is a no-op.  We do not
   * gate publishing on subscriber count because non-subscribers may
   * still poll GET_STATUS via ioctl.
   */

  UNUSED(lower);
  UNUSED(filep);
  UNUSED(enable);
  return OK;
}

static int legosensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep,
                                   FAR uint32_t *interval)
{
  /* The LUMP cadence is dictated by the device firmware; we do not
   * throttle here.  Accept any interval but keep the reported value.
   */

  UNUSED(lower);
  UNUSED(filep);
  UNUSED(interval);
  return OK;
}

static int legosensor_control(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              int cmd, unsigned long arg)
{
  FAR struct legosensor_dev_s *dev =
      container_of(lower, struct legosensor_dev_s, lower);
  int ret;

  switch (cmd)
    {
      case LEGOSENSOR_GET_INFO:
        {
          FAR struct lump_device_info_s *out =
              (FAR struct lump_device_info_s *)(uintptr_t)arg;
          if (out == NULL)
            {
              return -EINVAL;
            }

          /* Defer to the LUMP engine for the authoritative snapshot. */

          return lump_get_info(dev->port, out);
        }

      case LEGOSENSOR_GET_STATUS:
        {
          FAR struct lump_status_full_s *out =
              (FAR struct lump_status_full_s *)(uintptr_t)arg;
          if (out == NULL)
            {
              return -EINVAL;
            }

          return lump_get_status_full(dev->port, out);
        }

      case LEGOSENSOR_CLAIM:
        {
          nxmutex_lock(&dev->lock);
          if (dev->claim_owner != NULL && dev->claim_owner != filep)
            {
              ret = -EBUSY;
            }
          else
            {
              dev->claim_owner = filep;        /* idempotent for owner */
              ret = OK;
            }
          nxmutex_unlock(&dev->lock);
          return ret;
        }

      case LEGOSENSOR_RELEASE:
        {
          nxmutex_lock(&dev->lock);
          if (dev->claim_owner != filep)
            {
              ret = -EACCES;
            }
          else
            {
              dev->claim_owner = NULL;
              ret = OK;
            }
          nxmutex_unlock(&dev->lock);
          return ret;
        }

      case LEGOSENSOR_SELECT:
        {
          uint8_t mode = (uint8_t)arg;

          nxmutex_lock(&dev->lock);
          if (dev->claim_owner != filep)
            {
              nxmutex_unlock(&dev->lock);
              return -EACCES;
            }

          /* Arm the pending-SELECT tracker.  generation is bumped by
           * on_data() once the device starts reporting `mode`, or the
           * tracker is cleared if the deadline lapses first.
           */

          dev->pending_select_mode = mode;
          dev->pending_select_deadline = clock_systime_ticks() +
              MSEC2TICK(LEGOSENSOR_PENDING_TIMEOUT_MS);
          nxmutex_unlock(&dev->lock);

          ret = lump_select_mode(dev->port, mode);
          if (ret < 0)
            {
              /* LUMP rejected the request — clear the pending state so
               * a future SELECT does not race with stale data.
               */

              nxmutex_lock(&dev->lock);
              dev->pending_select_mode = LEGOSENSOR_NO_PENDING;
              nxmutex_unlock(&dev->lock);
            }

          return ret;
        }

      case LEGOSENSOR_SEND:
        {
          FAR const struct legosensor_send_arg_s *snd =
              (FAR const struct legosensor_send_arg_s *)(uintptr_t)arg;
          uint8_t  mode;
          uint8_t  len;

          if (snd == NULL)
            {
              return -EINVAL;
            }

          if (snd->len == 0 || snd->len > LUMP_MAX_PAYLOAD)
            {
              return -EINVAL;
            }

          mode = snd->mode;
          len  = snd->len;

          nxmutex_lock(&dev->lock);
          if (dev->claim_owner != filep)
            {
              nxmutex_unlock(&dev->lock);
              return -EACCES;
            }
          nxmutex_unlock(&dev->lock);

          /* Snapshot the payload onto the kthread's stack via lump_send_data
           * so the LUMP engine can hold its own copy.
           */

          return lump_send_data(dev->port, mode, snd->data, len);
        }

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Private Functions — registration helpers
 ****************************************************************************/

static int legosensor_register_port(FAR struct legosensor_dev_s *dev,
                                    int port)
{
  struct lump_callbacks_s cb;
  int ret;

  memset(dev, 0, sizeof(*dev));
  dev->port                 = port;
  dev->pending_select_mode  = LEGOSENSOR_NO_PENDING;
  dev->lower.type           = SENSOR_TYPE_CUSTOM;
  dev->lower.ops            = &g_legosensor_ops;
  dev->lower.nbuffer        = LEGOSENSOR_NBUFFER;
  nxmutex_init(&dev->lock);

  snprintf(dev->path, sizeof(dev->path),
           "/dev/uorb/sensor_lego%d", port);

  ret = sensor_custom_register(&dev->lower, dev->path,
                               sizeof(struct lump_sample_s));
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: sensor_custom_register(%s) failed: %d\n",
             dev->path, ret);
      nxmutex_destroy(&dev->lock);
      return ret;
    }

  dev->registered = true;

  /* Attach per-port LUMP callbacks.  Pass `dev` as `priv` so callbacks
   * can recover the per-port struct without needing port indexing.
   */

  cb       = g_legosensor_lump_cb;
  cb.priv  = dev;

  ret = lump_attach(port, &cb);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: lump_attach(%d) failed: %d\n", port, ret);
      sensor_custom_unregister(&dev->lower, dev->path);
      dev->registered = false;
      nxmutex_destroy(&dev->lock);
      return ret;
    }

  dev->attached = true;
  return OK;
}

static void legosensor_unregister_port(FAR struct legosensor_dev_s *dev)
{
  if (dev->attached)
    {
      lump_detach(dev->port);
      dev->attached = false;
    }

  if (dev->registered)
    {
      sensor_custom_unregister(&dev->lower, dev->path);
      dev->registered = false;
    }

  nxmutex_destroy(&dev->lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int legosensor_uorb_register(void)
{
  int ret;
  int i;
  int j;

  for (i = 0; i < LEGOSENSOR_NUM_PORTS; i++)
    {
      ret = legosensor_register_port(&g_legosensor[i], i);
      if (ret < 0)
        {
          /* Roll back any successful registrations so we leave a clean
           * state behind.  The bringup caller logs and continues so
           * other subsystems are not affected.
           */

          for (j = 0; j < i; j++)
            {
              legosensor_unregister_port(&g_legosensor[j]);
            }

          return ret;
        }
    }

  syslog(LOG_INFO,
         "legosensor: registered /dev/uorb/sensor_lego[0..5]\n");
  return OK;
}
