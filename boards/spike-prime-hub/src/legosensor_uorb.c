/****************************************************************************
 * boards/spike-prime-hub/src/legosensor_uorb.c
 *
 * LEGO Powered Up sensor uORB driver (Issue #79) — six device-class
 * topics under `/dev/uorb/sensor_*`.
 *
 * Six class topics are registered statically at boot.  Every physical
 * port runs an independent LUMP UART session (Issue #43) and feeds its
 * `on_sync` / `on_data` / `on_error` callbacks into a port→class
 * classifier.  Each class slot is bound to at most one port at any
 * instant; when two ports classify the same way the lower-numbered one
 * wins.  Frames from any port that lost a contention are dropped.  When
 * a port disconnects the class topic emits a disconnect sentinel and
 * releases its slot — no automatic rebind to other connected ports of
 * the same type, by design.
 *
 * Locking discipline:
 *   - g_bind_lock   protects the class-side bind state (bound_port,
 *                   bind_generation, claim_owner, claim_bind_gen).
 *   - per-port lock protects the port telemetry (seq, generation, info
 *                   cache, pending SELECT, light_mode_idx).
 *   - The two locks are NEVER held at the same time.  Any code path
 *                   that needs both must read one, drop it, then take
 *                   the other.
 *   - push_event() is invoked outside of every lock to avoid the upper
 *                   half nxrmutex_t cycling against our per-port lock.
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
#include <nuttx/mutex.h>
#include <nuttx/sensors/sensor.h>

#include <arch/board/board_legosensor.h>
#include <arch/board/board_lump.h>

#include "board_usercheck.h"

/* Forward declaration from stm32_legoport_pwm.c — motor SET_PWM
 * (LEGOSENSOR_CLASS_MOTOR_*) is plumbed straight through to the
 * H-bridge driver shipped in Issue #80.
 */

int stm32_legoport_pwm_set_duty(int idx, int16_t duty);
int stm32_legoport_pwm_coast(int idx);
int stm32_legoport_pwm_brake(int idx);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LEGOSENSOR_NO_PENDING        0xffu
#define LEGOSENSOR_PENDING_TIMEOUT_MS 500

/* LPF2 type IDs that this driver classifies (matches pybricks
 * `lego_uart.h`).  In the SPIKE Prime kit the Medium Motor (48) is
 * the lower-torque, faster motor used as the driving wheel and is
 * split into MOTOR_R / MOTOR_L by port parity; the Large Motor (49)
 * is the higher-torque motor used for arms / manipulators and routes
 * to MOTOR_M.
 */

#define LPF2_TYPE_COLOR              61
#define LPF2_TYPE_ULTRASONIC         62
#define LPF2_TYPE_FORCE              63
#define LPF2_TYPE_SPIKE_MEDIUM_MOTOR 48   /* driving wheel */
#define LPF2_TYPE_SPIKE_LARGE_MOTOR  49   /* arm / module motor */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct legosensor_class_dev_s
{
  struct sensor_lowerhalf_s lower;
  enum legosensor_class_e   class_id;
  bool                      registered;
  const char               *path;
};

/* Bind / claim state for one class slot.  Guarded by g_bind_lock. */

struct legosensor_class_state_s
{
  int               bound_port;        /* -1 if unbound, else 0..5 */
  uint32_t          bind_generation;   /* bumps on every bind change */
  FAR struct file  *claim_owner;       /* fd that holds CLAIM, or NULL */
  uint32_t          claim_bind_gen;    /* bind_generation at CLAIM time */
};

/* Per-port telemetry / cache.  Guarded by `lock`; bind ownership is
 * derived from g_class_state[*].bound_port instead of a back-pointer.
 */

struct legosensor_port_s
{
  mutex_t  lock;
  int      port;                       /* 0..5 */
  bool     attached;                   /* lump_attach succeeded */
  bool     synced;                     /* SYNC complete, info valid */
  bool     classify_warned;            /* throttle unmapped warnings */

  uint32_t seq;
  uint32_t generation;
  uint8_t  type_id;
  uint8_t  mode_id;
  uint8_t  pending_select_mode;
  clock_t  pending_select_deadline;

  int      light_mode_idx;             /* writable LIGHT mode (Phase 4) */

  struct lump_device_info_s info_cache;
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
  .priv     = NULL,             /* set per port at attach time */
};

static struct legosensor_class_dev_s
    g_legosensor_class[LEGOSENSOR_CLASS_NUM] =
{
  [LEGOSENSOR_CLASS_COLOR]      = { .class_id = LEGOSENSOR_CLASS_COLOR,
                                    .path     = "/dev/uorb/sensor_color"      },
  [LEGOSENSOR_CLASS_ULTRASONIC] = { .class_id = LEGOSENSOR_CLASS_ULTRASONIC,
                                    .path     = "/dev/uorb/sensor_ultrasonic" },
  [LEGOSENSOR_CLASS_FORCE]      = { .class_id = LEGOSENSOR_CLASS_FORCE,
                                    .path     = "/dev/uorb/sensor_force"      },
  [LEGOSENSOR_CLASS_MOTOR_M]    = { .class_id = LEGOSENSOR_CLASS_MOTOR_M,
                                    .path     = "/dev/uorb/sensor_motor_m"    },
  [LEGOSENSOR_CLASS_MOTOR_R]    = { .class_id = LEGOSENSOR_CLASS_MOTOR_R,
                                    .path     = "/dev/uorb/sensor_motor_r"    },
  [LEGOSENSOR_CLASS_MOTOR_L]    = { .class_id = LEGOSENSOR_CLASS_MOTOR_L,
                                    .path     = "/dev/uorb/sensor_motor_l"    },
};

static struct legosensor_class_state_s
    g_legosensor_class_state[LEGOSENSOR_CLASS_NUM];

static struct legosensor_port_s g_legosensor_port[LEGOSENSOR_NUM_PORTS];

static mutex_t g_bind_lock;

/****************************************************************************
 * Private Functions — classifier
 ****************************************************************************/

static enum legosensor_class_e legosensor_classify(uint8_t type_id, int port)
{
  switch (type_id)
    {
      case LPF2_TYPE_COLOR:
        return LEGOSENSOR_CLASS_COLOR;
      case LPF2_TYPE_ULTRASONIC:
        return LEGOSENSOR_CLASS_ULTRASONIC;
      case LPF2_TYPE_FORCE:
        return LEGOSENSOR_CLASS_FORCE;
      case LPF2_TYPE_SPIKE_LARGE_MOTOR:
        return LEGOSENSOR_CLASS_MOTOR_M;
      case LPF2_TYPE_SPIKE_MEDIUM_MOTOR:
        return ((port & 1) == 0) ? LEGOSENSOR_CLASS_MOTOR_R
                                 : LEGOSENSOR_CLASS_MOTOR_L;
      default:
        return LEGOSENSOR_CLASS_NONE;
    }
}

/****************************************************************************
 * Private Functions — envelope helpers
 ****************************************************************************/

/* Build a sample envelope from current port state and the given LUMP
 * frame.  Caller MUST hold p->lock.  The envelope is returned by value
 * so the caller can release the lock before push_event().
 */

static void legosensor_build_envelope(FAR struct legosensor_port_s *p,
                                      uint8_t mode,
                                      FAR const uint8_t *data, size_t len,
                                      FAR struct lump_sample_s *out)
{
  uint8_t  num_values = 0;
  uint8_t  data_type  = 0;
  size_t   copy_len   = len > LEGOSENSOR_MAX_DATA_BYTES ?
                        LEGOSENSOR_MAX_DATA_BYTES : len;

  if (p->synced && mode < p->info_cache.num_modes)
    {
      num_values = p->info_cache.modes[mode].num_values;
      data_type  = p->info_cache.modes[mode].data_type;
    }

  memset(out, 0, sizeof(*out));
  out->timestamp  = sensor_get_timestamp();
  out->seq        = p->seq;
  out->generation = p->generation;
  out->port       = (uint8_t)p->port;
  out->type_id    = p->type_id;
  out->mode_id    = mode;
  out->data_type  = data_type;
  out->num_values = num_values;
  out->len        = (uint8_t)copy_len;

  if (data != NULL && copy_len > 0)
    {
      memcpy(out->data.raw, data, copy_len);
    }
}

/* Build a synthetic disconnect sentinel for a port that just lost its
 * class binding (either real disconnect or displacement by a lower-
 * numbered port).  Caller MUST hold p->lock.  Bumps seq/generation as
 * a side-effect so subscribers see a generation jump on the topic.
 */

static void legosensor_build_disconnect_locked(FAR struct legosensor_port_s *p,
                                               FAR struct lump_sample_s *out)
{
  p->seq++;
  p->generation++;

  memset(out, 0, sizeof(*out));
  out->timestamp  = sensor_get_timestamp();
  out->seq        = p->seq;
  out->generation = p->generation;
  out->port       = (uint8_t)p->port;
  /* type_id, mode_id, data_type, num_values, len already zero
   * → disconnect sentinel encoding.
   */
}

/****************************************************************************
 * Private Functions — LIGHT mode resolution (Phase 4 prep)
 ****************************************************************************/

/* Locate the writable "LIGHT" mode that matches the class's expected
 * shape (3 × INT8 for color, 4 × INT8 for ultrasonic).  Returns the
 * mode index or -1 if not present / wrong shape.  Caller MUST hold
 * p->lock with info_cache populated.
 */

static int legosensor_resolve_light_locked(FAR struct legosensor_port_s *p,
                                           enum legosensor_class_e cls)
{
  uint8_t want_values;

  switch (cls)
    {
      case LEGOSENSOR_CLASS_COLOR:      want_values = 3; break;
      case LEGOSENSOR_CLASS_ULTRASONIC: want_values = 4; break;
      default:                          return -1;
    }

  for (int m = 0; m < p->info_cache.num_modes && m < LUMP_MAX_MODES; m++)
    {
      const struct lump_mode_info_s *mi = &p->info_cache.modes[m];
      if (mi->writable &&
          mi->data_type == LUMP_DATA_INT8 &&
          mi->num_values == want_values &&
          strncmp(mi->name, "LIGHT", LUMP_MAX_NAME_LEN) == 0)
        {
          return m;
        }
    }

  return -1;
}

/****************************************************************************
 * Private Functions — bind management (g_bind_lock)
 ****************************************************************************/

/* Attempt to bind `port` to its classified slot.  Returns the new bind
 * outcome and, on a successful bind that displaced a previous owner,
 * sets *displaced_port to the previous port (else -1).  Caller is
 * responsible for emitting the disconnect / sync sentinels outside the
 * lock.
 */

enum legosensor_bind_result_e
{
  BIND_OUTCOME_BOUND_FRESH = 0,        /* slot was empty, we own it */
  BIND_OUTCOME_BOUND_DISPLACED,        /* we displaced a higher-numbered port */
  BIND_OUTCOME_LOSS,                   /* lower-numbered port already owns */
  BIND_OUTCOME_NONE,                   /* unmapped class, no bind */
};

static enum legosensor_bind_result_e
legosensor_try_bind(int port, enum legosensor_class_e cls,
                    FAR int *displaced_port)
{
  FAR struct legosensor_class_state_s *cs;
  enum legosensor_bind_result_e ret;

  *displaced_port = -1;

  if (cls == LEGOSENSOR_CLASS_NONE)
    {
      return BIND_OUTCOME_NONE;
    }

  cs = &g_legosensor_class_state[cls];

  nxmutex_lock(&g_bind_lock);

  if (cs->bound_port < 0)
    {
      cs->bound_port = port;
      cs->bind_generation++;
      ret = BIND_OUTCOME_BOUND_FRESH;
    }
  else if (cs->bound_port > port)
    {
      *displaced_port = cs->bound_port;
      cs->bound_port = port;
      cs->bind_generation++;
      ret = BIND_OUTCOME_BOUND_DISPLACED;
    }
  else
    {
      ret = BIND_OUTCOME_LOSS;
    }

  /* claim_owner / claim_bind_gen are intentionally left alone here:
   * the bind_generation bump is what tells a stale fd that its CLAIM
   * was invalidated (write ioctls fail with -ENODEV until re-CLAIM).
   * close() of the stale fd clears the record opportunistically; a
   * new CLAIM from any fd overwrites a stale record automatically.
   */

  nxmutex_unlock(&g_bind_lock);
  return ret;
}

/* Release `port`'s grip on whichever class slot it currently holds,
 * if any.  Returns the displaced class (or LEGOSENSOR_CLASS_NONE if
 * the port was not bound).  Bumps bind_generation on the class so any
 * pending CLAIM held by a fd is implicitly invalidated.
 */

static enum legosensor_class_e legosensor_unbind(int port)
{
  enum legosensor_class_e cls = LEGOSENSOR_CLASS_NONE;
  int i;

  nxmutex_lock(&g_bind_lock);
  for (i = 0; i < LEGOSENSOR_CLASS_NUM; i++)
    {
      if (g_legosensor_class_state[i].bound_port == port)
        {
          cls = (enum legosensor_class_e)i;
          g_legosensor_class_state[i].bound_port = -1;
          g_legosensor_class_state[i].bind_generation++;
          /* claim_owner left alone — see legosensor_try_bind comment. */
          break;
        }
    }
  nxmutex_unlock(&g_bind_lock);

  return cls;
}

/* Look up which class is currently bound to `port`, or NONE. */

static enum legosensor_class_e legosensor_class_of(int port)
{
  enum legosensor_class_e cls = LEGOSENSOR_CLASS_NONE;
  int i;

  nxmutex_lock(&g_bind_lock);
  for (i = 0; i < LEGOSENSOR_CLASS_NUM; i++)
    {
      if (g_legosensor_class_state[i].bound_port == port)
        {
          cls = (enum legosensor_class_e)i;
          break;
        }
    }
  nxmutex_unlock(&g_bind_lock);

  return cls;
}

/****************************************************************************
 * Private Functions — LUMP callbacks (per-port kthread context)
 ****************************************************************************/

static void legosensor_on_sync(int port,
                               FAR const struct lump_device_info_s *info,
                               FAR void *priv)
{
  FAR struct legosensor_port_s *p = (FAR struct legosensor_port_s *)priv;
  enum legosensor_class_e cls;
  enum legosensor_bind_result_e bres;
  int displaced_port = -1;
  struct lump_sample_s sentinel;

  if (p == NULL || info == NULL)
    {
      return;
    }

  /* Phase A — populate per-port telemetry. */

  nxmutex_lock(&p->lock);
  memcpy(&p->info_cache, info, sizeof(p->info_cache));
  p->synced              = true;
  p->type_id             = info->type_id;
  p->mode_id             = info->current_mode;
  p->pending_select_mode = LEGOSENSOR_NO_PENDING;
  p->classify_warned     = false;
  p->seq++;
  p->generation++;
  cls = legosensor_classify(info->type_id, port);
  p->light_mode_idx = legosensor_resolve_light_locked(p, cls);
  nxmutex_unlock(&p->lock);

  /* Phase B — bind / contention resolution. */

  if (cls == LEGOSENSOR_CLASS_NONE)
    {
      if (!p->classify_warned)
        {
          syslog(LOG_INFO,
                 "legosensor: port %c type %u unmapped, frames dropped\n",
                 'A' + port, info->type_id);
          p->classify_warned = true;
        }
      return;
    }

  bres = legosensor_try_bind(port, cls, &displaced_port);

  /* Phase C — push sentinels outside of all locks. */

  if (bres == BIND_OUTCOME_BOUND_DISPLACED && displaced_port >= 0)
    {
      FAR struct legosensor_port_s *dp =
          &g_legosensor_port[displaced_port];
      struct lump_sample_s disc;

      nxmutex_lock(&dp->lock);
      legosensor_build_disconnect_locked(dp, &disc);
      nxmutex_unlock(&dp->lock);

      g_legosensor_class[cls].lower.push_event(
          g_legosensor_class[cls].lower.priv, &disc, sizeof(disc));
    }

  if (bres == BIND_OUTCOME_BOUND_FRESH ||
      bres == BIND_OUTCOME_BOUND_DISPLACED)
    {
      nxmutex_lock(&p->lock);
      legosensor_build_envelope(p, p->mode_id, NULL, 0, &sentinel);
      nxmutex_unlock(&p->lock);

      g_legosensor_class[cls].lower.push_event(
          g_legosensor_class[cls].lower.priv, &sentinel, sizeof(sentinel));
    }
}

static void legosensor_on_data(int port, uint8_t mode,
                               FAR const uint8_t *data, size_t len,
                               FAR void *priv)
{
  FAR struct legosensor_port_s *p = (FAR struct legosensor_port_s *)priv;
  struct lump_sample_s sample;
  enum legosensor_class_e cls;
  bool need_hydrate;
  bool fire_sync_sentinel = false;
  struct lump_sample_s sentinel;
  clock_t now;

  if (p == NULL)
    {
      return;
    }

  /* Defensive lazy info_cache hydration.  Issue #76 made the LUMP engine
   * fire `on_sync` on every SYNCING -> DATA edge, but keep the backstop
   * in case a sync was lost during a tight reconnect race.  No-op when
   * the cache already matches the engine state.
   */

  nxmutex_lock(&p->lock);
  need_hydrate = !p->synced || p->info_cache.num_modes == 0;
  nxmutex_unlock(&p->lock);

  if (need_hydrate)
    {
      struct lump_device_info_s info;

      if (lump_get_info(port, &info) == OK)
        {
          enum legosensor_class_e new_cls;

          nxmutex_lock(&p->lock);
          if (info.type_id != p->info_cache.type_id ||
              info.num_modes != p->info_cache.num_modes)
            {
              memcpy(&p->info_cache, &info, sizeof(p->info_cache));
              p->type_id = info.type_id;
              p->mode_id = info.current_mode;
              p->generation++;
              p->seq++;
              p->synced = true;
              new_cls = legosensor_classify(info.type_id, port);
              p->light_mode_idx = legosensor_resolve_light_locked(p, new_cls);
              fire_sync_sentinel = true;

              legosensor_build_envelope(p, p->mode_id, NULL, 0, &sentinel);
            }
          else if (!p->synced)
            {
              p->synced = true;
            }
          nxmutex_unlock(&p->lock);
        }
    }

  /* Look up the current class binding for this port.  Need to do it
   * without holding the per-port lock to keep the strict bind→port
   * lock-order rule.
   */

  cls = legosensor_class_of(port);

  if (fire_sync_sentinel && cls != LEGOSENSOR_CLASS_NONE)
    {
      g_legosensor_class[cls].lower.push_event(
          g_legosensor_class[cls].lower.priv, &sentinel, sizeof(sentinel));
    }

  if (cls == LEGOSENSOR_CLASS_NONE)
    {
      /* Either unmapped or this port lost its bind contention; drop. */

      return;
    }

  /* Phase D — build + push the data envelope. */

  nxmutex_lock(&p->lock);

  /* Generation update on confirmed SELECT. */

  if (p->pending_select_mode != LEGOSENSOR_NO_PENDING)
    {
      now = clock_systime_ticks();
      if ((int32_t)(now - p->pending_select_deadline) >= 0)
        {
          p->pending_select_mode = LEGOSENSOR_NO_PENDING;
        }
      else if (mode == p->pending_select_mode)
        {
          p->generation++;
          p->pending_select_mode = LEGOSENSOR_NO_PENDING;
        }
    }

  p->mode_id = mode;
  p->seq++;
  legosensor_build_envelope(p, mode, data, len, &sample);

  nxmutex_unlock(&p->lock);

  g_legosensor_class[cls].lower.push_event(
      g_legosensor_class[cls].lower.priv, &sample, sizeof(sample));
}

static void legosensor_on_error(int port, uint8_t mode,
                                FAR const uint8_t *data, size_t len,
                                FAR void *priv)
{
  FAR struct legosensor_port_s *p = (FAR struct legosensor_port_s *)priv;
  enum legosensor_class_e cls;
  struct lump_sample_s sentinel;

  UNUSED(mode);
  UNUSED(data);
  UNUSED(len);

  if (p == NULL)
    {
      return;
    }

  /* Drop any class binding first (under bind_lock alone). */

  cls = legosensor_unbind(port);

  /* Reset port telemetry under per-port lock and build the disconnect
   * sentinel from the post-reset state.
   */

  nxmutex_lock(&p->lock);

  p->synced              = false;
  p->type_id             = 0;
  p->mode_id             = 0;
  p->pending_select_mode = LEGOSENSOR_NO_PENDING;
  p->light_mode_idx      = -1;
  p->classify_warned     = false;
  p->seq++;
  p->generation++;
  memset(&p->info_cache, 0, sizeof(p->info_cache));

  memset(&sentinel, 0, sizeof(sentinel));
  sentinel.timestamp  = sensor_get_timestamp();
  sentinel.seq        = p->seq;
  sentinel.generation = p->generation;
  sentinel.port       = (uint8_t)p->port;
  /* type_id == 0, len == 0 → disconnect encoding */

  nxmutex_unlock(&p->lock);

  if (cls != LEGOSENSOR_CLASS_NONE)
    {
      g_legosensor_class[cls].lower.push_event(
          g_legosensor_class[cls].lower.priv, &sentinel, sizeof(sentinel));
    }
}

/****************************************************************************
 * Private Functions — claim helpers (g_bind_lock)
 ****************************************************************************/

/* Validate a write-side ioctl: caller must hold the CLAIM AND the
 * CLAIM must have been taken against the current bind_generation.
 * On success, *bound_port_out is set to the bound port.
 *
 * Returns:
 *   OK       - caller may proceed; *bound_port_out is the target port
 *   -ENODEV  - no port bound, OR caller's claim is stale (port was
 *              hot-swapped or disconnected since CLAIM)
 *   -EACCES  - caller does not hold the CLAIM
 */

static int legosensor_check_write_claim(
    FAR struct legosensor_class_state_s *cs,
    FAR struct file *filep,
    FAR int *bound_port_out)
{
  int ret;

  nxmutex_lock(&g_bind_lock);
  if (cs->bound_port < 0)
    {
      ret = -ENODEV;
    }
  else if (cs->claim_owner == filep)
    {
      if (cs->claim_bind_gen != cs->bind_generation)
        {
          /* Stale claim — port was rebound since the CLAIM. */

          ret = -ENODEV;
        }
      else
        {
          *bound_port_out = cs->bound_port;
          ret = OK;
        }
    }
  else
    {
      /* Different fd holds the claim, or no fd holds it. */

      ret = -EACCES;
    }
  nxmutex_unlock(&g_bind_lock);

  return ret;
}

/****************************************************************************
 * Private Functions — sensor_ops_s
 ****************************************************************************/

static int legosensor_open(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep)
{
  UNUSED(lower);

  if ((filep->f_oflags & O_WROK) != 0)
    {
      return -EACCES;
    }

  return OK;
}

static int legosensor_close(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep)
{
  FAR struct legosensor_class_dev_s *cdev =
      container_of(lower, struct legosensor_class_dev_s, lower);
  FAR struct legosensor_class_state_s *cs;

  if ((unsigned)cdev->class_id >= LEGOSENSOR_CLASS_NUM)
    {
      return OK;
    }

  cs = &g_legosensor_class_state[cdev->class_id];

  /* Auto-RELEASE if this fd held the CLAIM. */

  nxmutex_lock(&g_bind_lock);
  if (cs->claim_owner == filep)
    {
      cs->claim_owner    = NULL;
      cs->claim_bind_gen = 0;
    }
  nxmutex_unlock(&g_bind_lock);

  return OK;
}

static int legosensor_activate(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep, bool enable)
{
  UNUSED(lower);
  UNUSED(filep);
  UNUSED(enable);
  return OK;
}

static int legosensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep,
                                   FAR uint32_t *interval)
{
  UNUSED(lower);
  UNUSED(filep);
  UNUSED(interval);
  return OK;
}

static int legosensor_control(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              int cmd, unsigned long arg)
{
  FAR struct legosensor_class_dev_s *cdev =
      container_of(lower, struct legosensor_class_dev_s, lower);
  FAR struct legosensor_class_state_s *cs;
  int bound_port;
  int ret;

  UNUSED(filep);
  UNUSED(arg);

  if ((unsigned)cdev->class_id >= LEGOSENSOR_CLASS_NUM)
    {
      return -ENOTTY;
    }

  cs = &g_legosensor_class_state[cdev->class_id];

  /* Snapshot the bound port under bind_lock; fall straight through to
   * -ENODEV if the slot is currently empty.
   */

  nxmutex_lock(&g_bind_lock);
  bound_port = cs->bound_port;
  nxmutex_unlock(&g_bind_lock);

  switch (cmd)
    {
      case LEGOSENSOR_GET_INFO:
        {
          FAR struct legosensor_info_arg_s *out =
              (FAR struct legosensor_info_arg_s *)(uintptr_t)arg;

          if (out == NULL)
            {
              return -EINVAL;
            }

          if (!board_user_out_ok(out, sizeof(*out)))
            {
              return -EFAULT;
            }

          if (bound_port < 0)
            {
              return -ENODEV;
            }

          memset(out, 0, sizeof(*out));
          out->port = (uint8_t)bound_port;

          ret = lump_get_info(bound_port, &out->info);
          return ret;
        }

      case LEGOSENSOR_GET_STATUS:
        {
          FAR struct lump_status_full_s *out =
              (FAR struct lump_status_full_s *)(uintptr_t)arg;

          if (out == NULL)
            {
              return -EINVAL;
            }

          if (!board_user_out_ok(out, sizeof(*out)))
            {
              return -EFAULT;
            }

          if (bound_port < 0)
            {
              return -ENODEV;
            }

          return lump_get_status_full(bound_port, out);
        }

      case LEGOSENSOR_CLAIM:
        {
          nxmutex_lock(&g_bind_lock);

          if (cs->bound_port < 0)
            {
              ret = -ENODEV;
            }
          else if (cs->claim_owner == filep &&
                   cs->claim_bind_gen == cs->bind_generation)
            {
              /* Idempotent re-CLAIM by the existing owner. */

              ret = OK;
            }
          else if (cs->claim_owner == NULL ||
                   cs->claim_bind_gen != cs->bind_generation)
            {
              /* Slot is free OR previous holder's claim is stale —
               * either way, we may take ownership.
               */

              cs->claim_owner    = filep;
              cs->claim_bind_gen = cs->bind_generation;
              ret = OK;
            }
          else
            {
              ret = -EBUSY;
            }

          nxmutex_unlock(&g_bind_lock);
          return ret;
        }

      case LEGOSENSOR_RELEASE:
        {
          nxmutex_lock(&g_bind_lock);

          if (cs->claim_owner != filep ||
              cs->claim_bind_gen != cs->bind_generation)
            {
              ret = -EACCES;
            }
          else
            {
              cs->claim_owner    = NULL;
              cs->claim_bind_gen = 0;
              ret = OK;
            }

          nxmutex_unlock(&g_bind_lock);
          return ret;
        }

      case LEGOSENSOR_SELECT:
        {
          FAR const struct legosensor_select_arg_s *sel =
              (FAR const struct legosensor_select_arg_s *)(uintptr_t)arg;
          struct legosensor_select_arg_s sel_kern;
          FAR struct legosensor_port_s *p;
          uint8_t mode;
          int port_to_use = -1;

          if (sel == NULL)
            {
              return -EINVAL;
            }

          if (!board_user_in_ok(sel, sizeof(*sel)))
            {
              return -EFAULT;
            }

          memcpy(&sel_kern, sel, sizeof(sel_kern));
          mode = sel_kern.mode;

          ret = legosensor_check_write_claim(cs, filep, &port_to_use);
          if (ret < 0)
            {
              return ret;
            }

          /* Arm the pending-SELECT tracker on the bound port so the
           * next on_data with `mode == requested` bumps generation.
           * Must be done before lump_select_mode() to avoid a race
           * where the device replies before the tracker is armed.
           */

          p = &g_legosensor_port[port_to_use];
          nxmutex_lock(&p->lock);
          p->pending_select_mode     = mode;
          p->pending_select_deadline = clock_systime_ticks() +
              MSEC2TICK(LEGOSENSOR_PENDING_TIMEOUT_MS);
          nxmutex_unlock(&p->lock);

          ret = lump_select_mode(port_to_use, mode);
          if (ret < 0)
            {
              nxmutex_lock(&p->lock);
              p->pending_select_mode = LEGOSENSOR_NO_PENDING;
              nxmutex_unlock(&p->lock);
            }

          return ret;
        }

      case LEGOSENSOR_SEND:
        {
          FAR const struct legosensor_send_arg_s *snd =
              (FAR const struct legosensor_send_arg_s *)(uintptr_t)arg;
          struct legosensor_send_arg_s snd_kern;
          int port_to_use = -1;

          if (snd == NULL)
            {
              return -EINVAL;
            }

          if (!board_user_in_ok(snd, sizeof(*snd)))
            {
              return -EFAULT;
            }

          /* Copy into kernel-local before validating len: a concurrent
           * user-side mutation between the len check and lump_send_data
           * could otherwise let len exceed the validated bound.
           */

          memcpy(&snd_kern, snd, sizeof(snd_kern));

          if (snd_kern.len == 0 || snd_kern.len > LUMP_MAX_PAYLOAD)
            {
              return -EINVAL;
            }

          ret = legosensor_check_write_claim(cs, filep, &port_to_use);
          if (ret < 0)
            {
              return ret;
            }

          return lump_send_data(port_to_use, snd_kern.mode,
                                snd_kern.data, snd_kern.len);
        }

      case LEGOSENSOR_SET_PWM:
        {
          FAR const struct legosensor_pwm_arg_s *pwm =
              (FAR const struct legosensor_pwm_arg_s *)(uintptr_t)arg;
          struct legosensor_pwm_arg_s pwm_kern;
          int port_to_use = -1;

          if (pwm == NULL)
            {
              return -EINVAL;
            }

          if (!board_user_in_ok(pwm, sizeof(*pwm)))
            {
              return -EFAULT;
            }

          /* Copy into kernel-local so num_channels / channels[] cannot
           * be mutated from user mode after the validation below.
           */

          memcpy(&pwm_kern, pwm, sizeof(pwm_kern));

          /* SET_PWM is reserved for **H-bridge physical PWM**, i.e. the
           * MOTOR_* classes routed through `stm32_legoport_pwm_set_duty`
           * (Issue #80).  COLOR / ULTRASONIC LIGHT modes and FORCE's
           * absence of any actuator are not PWM in the H-bridge sense,
           * so they return -ENOTSUP — callers must use `LEGOSENSOR_SEND
           * mode=LIGHT` directly (Issue #92).  Routing LED brightness
           * through SEND keeps the SELECT-then-WRITE sequence visible
           * and matches the underlying LUMP semantics.
           */

          switch (cdev->class_id)
            {
              case LEGOSENSOR_CLASS_MOTOR_M:
              case LEGOSENSOR_CLASS_MOTOR_R:
              case LEGOSENSOR_CLASS_MOTOR_L:
                /* num_channels must be 1; channels[0] = signed duty in
                 * -10000..10000 (.01 % units), exactly the
                 * `stm32_legoport_pwm_set_duty(idx, int16_t)` ABI.
                 */

                if (pwm_kern.num_channels != 1)
                  {
                    return -EINVAL;
                  }
                if (pwm_kern.channels[0] < -10000 ||
                    pwm_kern.channels[0] > 10000)
                  {
                    return -EINVAL;
                  }

                ret = legosensor_check_write_claim(cs, filep, &port_to_use);
                if (ret < 0)
                  {
                    return ret;
                  }

                /* -EBUSY surfaces if LUMP has the port pinned as a
                 * SUPPLY rail (only happens for COLOR / ULTRASONIC,
                 * which never reach this branch).  Motors do not
                 * announce NEEDS_SUPPLY_PIN, so this should pass.
                 */

                return stm32_legoport_pwm_set_duty(port_to_use,
                                                   pwm_kern.channels[0]);

              case LEGOSENSOR_CLASS_COLOR:
              case LEGOSENSOR_CLASS_ULTRASONIC:
              case LEGOSENSOR_CLASS_FORCE:
              default:
                /* COLOR mode 3 LIGHT and ULTRASONIC mode 5 LIGHT are
                 * firmware-gated per SELECT mode (see sensor.md §4.5
                 * for COLOR; ULTRASONIC behaves the same way per
                 * pybricks reference).  FORCE has no actuator at all.
                 * Validate CLAIM so the caller's stale-claim
                 * diagnostic still surfaces, then return -ENOTSUP.
                 * Use `LEGOSENSOR_SEND` (mode=LIGHT) to drive the LEDs
                 * explicitly.
                 */

                ret = legosensor_check_write_claim(cs, filep, &port_to_use);
                if (ret < 0)
                  {
                    return ret;
                  }
                return -ENOTSUP;
            }
        }

      case LEGOSENSOR_MOTOR_M_COAST:
      case LEGOSENSOR_MOTOR_M_BRAKE:
      case LEGOSENSOR_MOTOR_R_COAST:
      case LEGOSENSOR_MOTOR_R_BRAKE:
      case LEGOSENSOR_MOTOR_L_COAST:
      case LEGOSENSOR_MOTOR_L_BRAKE:
        {
          /* Per-class motor coast / brake.  CLAIM required.  Each motor
           * class owns its own ioctl number in the reserved range so the
           * dispatch can verify the caller's `class_id` matches the
           * requested base — guarantees an fd opened on the wrong class
           * cannot drive an unrelated motor by mistake.
           */

          int port_to_use = -1;
          enum legosensor_class_e expected_class;
          bool is_brake;

          if (cmd == LEGOSENSOR_MOTOR_M_COAST ||
              cmd == LEGOSENSOR_MOTOR_M_BRAKE)
            {
              expected_class = LEGOSENSOR_CLASS_MOTOR_M;
            }
          else if (cmd == LEGOSENSOR_MOTOR_R_COAST ||
                   cmd == LEGOSENSOR_MOTOR_R_BRAKE)
            {
              expected_class = LEGOSENSOR_CLASS_MOTOR_R;
            }
          else
            {
              expected_class = LEGOSENSOR_CLASS_MOTOR_L;
            }

          if (cdev->class_id != expected_class)
            {
              return -ENOTTY;
            }

          is_brake = (cmd == LEGOSENSOR_MOTOR_M_BRAKE ||
                      cmd == LEGOSENSOR_MOTOR_R_BRAKE ||
                      cmd == LEGOSENSOR_MOTOR_L_BRAKE);

          ret = legosensor_check_write_claim(cs, filep, &port_to_use);
          if (ret < 0)
            {
              return ret;
            }

          return is_brake ? stm32_legoport_pwm_brake(port_to_use)
                          : stm32_legoport_pwm_coast(port_to_use);
        }

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Private Functions — registration helpers
 ****************************************************************************/

static int legosensor_register_class(FAR struct legosensor_class_dev_s *dev)
{
  int ret;

  dev->lower.type    = SENSOR_TYPE_CUSTOM;
  dev->lower.ops     = &g_legosensor_ops;
  dev->lower.nbuffer = LEGOSENSOR_NBUFFER;

  ret = sensor_custom_register(&dev->lower, dev->path,
                               sizeof(struct lump_sample_s));
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: sensor_custom_register(%s) failed: %d\n",
             dev->path, ret);
      return ret;
    }

  dev->registered = true;
  return OK;
}

static void legosensor_unregister_class(FAR struct legosensor_class_dev_s *dev)
{
  if (dev->registered)
    {
      sensor_custom_unregister(&dev->lower, dev->path);
      dev->registered = false;
    }
}

static int legosensor_attach_port(FAR struct legosensor_port_s *p, int port)
{
  struct lump_callbacks_s cb;
  int ret;

  memset(p, 0, sizeof(*p));
  p->port                 = port;
  p->pending_select_mode  = LEGOSENSOR_NO_PENDING;
  p->light_mode_idx       = -1;
  nxmutex_init(&p->lock);

  cb       = g_legosensor_lump_cb;
  cb.priv  = p;

  ret = lump_attach(port, &cb);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lump_attach(%d) failed: %d\n", port, ret);
      nxmutex_destroy(&p->lock);
      return ret;
    }

  p->attached = true;
  return OK;
}

static void legosensor_detach_port(FAR struct legosensor_port_s *p)
{
  if (p->attached)
    {
      lump_detach(p->port);
      p->attached = false;
    }

  nxmutex_destroy(&p->lock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int legosensor_uorb_register(void)
{
  int ret;
  int i;
  int j;

  /* Initialise class-side bind state and the bind lock. */

  nxmutex_init(&g_bind_lock);
  for (i = 0; i < LEGOSENSOR_CLASS_NUM; i++)
    {
      g_legosensor_class_state[i].bound_port      = -1;
      g_legosensor_class_state[i].bind_generation = 0;
      g_legosensor_class_state[i].claim_owner     = NULL;
      g_legosensor_class_state[i].claim_bind_gen  = 0;
    }

  /* (1) Register all class topics first so push_event() targets exist
   * when lump_attach() fires a synchronous on_sync.
   */

  for (i = 0; i < LEGOSENSOR_CLASS_NUM; i++)
    {
      ret = legosensor_register_class(&g_legosensor_class[i]);
      if (ret < 0)
        {
          for (j = 0; j < i; j++)
            {
              legosensor_unregister_class(&g_legosensor_class[j]);
            }

          nxmutex_destroy(&g_bind_lock);
          return ret;
        }
    }

  /* (2) Attach LUMP callbacks per port. */

  for (i = 0; i < LEGOSENSOR_NUM_PORTS; i++)
    {
      ret = legosensor_attach_port(&g_legosensor_port[i], i);
      if (ret < 0)
        {
          for (j = 0; j < i; j++)
            {
              legosensor_detach_port(&g_legosensor_port[j]);
            }

          for (j = 0; j < LEGOSENSOR_CLASS_NUM; j++)
            {
              legosensor_unregister_class(&g_legosensor_class[j]);
            }

          nxmutex_destroy(&g_bind_lock);
          return ret;
        }
    }

  syslog(LOG_INFO,
         "legosensor: registered class topics + attached %d ports\n",
         LEGOSENSOR_NUM_PORTS);
  return OK;
}
