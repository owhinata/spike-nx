/****************************************************************************
 * apps/btsensor/sensor_sampler.c
 *
 * LEGO Powered Up sensor uORB drain helper for the BUNDLE emitter
 * (Issue #90 — full implementation; Issue #88 shipped a stub).
 *
 * Subscribes to the six device-class topics under /dev/uorb/sensor_*
 * (color/ultrasonic/force/motor_m/motor_r/motor_l), registers each fd
 * as a btstack data source, and caches the latest publish per class.
 * `sensor_sampler_snapshot()` copies that cache into the layout
 * bundle_emitter serialises into the BUNDLE TLV section.
 *
 * Issue B (this commit) is read-only — `LEGOSENSOR_CLAIM` and the write
 * APIs (SELECT / SEND / SET_PWM) are deferred to Issue C, which keeps
 * the read fds open and opens transient fds for writes.
 *
 * Class index in this module matches `enum legosensor_class_e` so
 * bundle_emitter can iterate by index.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include <arch/board/board_legosensor.h>

#include "btstack_event.h"
#include "btstack_run_loop.h"

#include "btsensor_wire.h"
#include "sensor_sampler.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Class table: index = enum legosensor_class_e value. */

struct class_table_s
{
  const char *path;
};

static const struct class_table_s g_class_table[BTSENSOR_TLV_COUNT] =
{
  [LEGOSENSOR_CLASS_COLOR]      = { "/dev/uorb/sensor_color"      },
  [LEGOSENSOR_CLASS_ULTRASONIC] = { "/dev/uorb/sensor_ultrasonic" },
  [LEGOSENSOR_CLASS_FORCE]      = { "/dev/uorb/sensor_force"      },
  [LEGOSENSOR_CLASS_MOTOR_M]    = { "/dev/uorb/sensor_motor_m"    },
  [LEGOSENSOR_CLASS_MOTOR_R]    = { "/dev/uorb/sensor_motor_r"    },
  [LEGOSENSOR_CLASS_MOTOR_L]    = { "/dev/uorb/sensor_motor_l"    },
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Per-class internal state.  `fresh` is set on each non-sentinel publish
 * and cleared by snapshot(); `bound` tracks the last seen BOUND/UNBOUND
 * transition.  `last_publish_ms` is CLOCK_BOOTTIME ms used to compute
 * age_10ms saturating at 0xFF (~2.55 s).
 */

struct class_state_s
{
  int                   fd;
  btstack_data_source_t ds;
  uint8_t               class_id;     /* same as table index, kept for ds lookup */
  uint8_t               port_id;      /* 0..5 when bound, 0xFF otherwise */
  uint8_t               mode_id;
  uint8_t               data_type;
  uint8_t               num_values;
  uint8_t               payload_len;
  uint8_t               payload[BTSENSOR_TLV_PAYLOAD_MAX];
  uint16_t              seq;
  bool                  bound;
  bool                  fresh;
  uint32_t              last_publish_ms;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct class_state_s g_classes[BTSENSOR_TLV_COUNT];
static bool                 g_initialized;
static bool                 g_enabled;

/* Per-class 50 ms cool-down for write APIs (Codex review #7).  Updated
 * on every successful select/send/set_pwm.
 */

#define SENSOR_WRITE_COOLDOWN_MS  50

static uint32_t             g_last_write_ms[BTSENSOR_TLV_COUNT];

/* Per-class PWM channel count, matching apps/sensor/sensor_main.c's
 * pwm_channels.  0 means SET_PWM is not supported on that class.
 */

static const uint8_t g_pwm_channels[BTSENSOR_TLV_COUNT] =
{
  [LEGOSENSOR_CLASS_COLOR]      = 3,
  [LEGOSENSOR_CLASS_ULTRASONIC] = 4,
  [LEGOSENSOR_CLASS_FORCE]      = 0,
  [LEGOSENSOR_CLASS_MOTOR_M]    = 1,
  [LEGOSENSOR_CLASS_MOTOR_R]    = 1,
  [LEGOSENSOR_CLASS_MOTOR_L]    = 1,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t now_ms(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_BOOTTIME, &ts);
  return (uint32_t)((uint64_t)ts.tv_sec * 1000U +
                    (uint64_t)ts.tv_nsec / 1000000U);
}

static void class_init(struct class_state_s *cls, uint8_t class_id)
{
  memset(cls, 0, sizeof(*cls));
  cls->fd       = -1;
  cls->class_id = class_id;
  cls->port_id  = 0xFF;
}

static struct class_state_s *class_from_ds(btstack_data_source_t *ds)
{
  for (size_t i = 0; i < BTSENSOR_TLV_COUNT; i++)
    {
      if (&g_classes[i].ds == ds)
        {
          return &g_classes[i];
        }
    }

  return NULL;
}

static void update_from_sample(struct class_state_s *cls,
                               const struct lump_sample_s *s)
{
  cls->seq             = (uint16_t)(s->seq & 0xFFFFU);
  cls->mode_id         = s->mode_id;
  cls->data_type       = s->data_type;
  cls->num_values      = s->num_values;
  cls->last_publish_ms = now_ms();

  if (s->type_id == 0 && s->len == 0)
    {
      /* Disconnect sentinel — port unbound. */

      cls->bound       = false;
      cls->port_id     = 0xFF;
      cls->payload_len = 0;
      cls->fresh       = false;
      return;
    }

  cls->bound   = true;
  cls->port_id = s->port;

  if (s->len == 0)
    {
      /* SYNC sentinel — bound but no payload yet. */

      cls->payload_len = 0;
      cls->fresh       = false;
      return;
    }

  /* Normal data sample. */

  uint8_t len = s->len;
  if (len > BTSENSOR_TLV_PAYLOAD_MAX)
    {
      len = BTSENSOR_TLV_PAYLOAD_MAX;
    }

  cls->payload_len = len;
  memcpy(cls->payload, s->data.raw, len);
  cls->fresh = true;
}

static void on_read(btstack_data_source_t *ds,
                    btstack_data_source_callback_type_t type)
{
  (void)type;

  struct class_state_s *cls = class_from_ds(ds);
  if (cls == NULL || cls->fd < 0)
    {
      return;
    }

  while (1)
    {
      struct lump_sample_s s;
      ssize_t n = read(cls->fd, &s, sizeof(s));
      if (n != sizeof(s))
        {
          break;
        }

      update_from_sample(cls, &s);
    }
}

static int class_open(struct class_state_s *cls)
{
  if (cls->fd >= 0)
    {
      return 0;
    }

  int fd = open(g_class_table[cls->class_id].path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      syslog(LOG_ERR, "btsensor: open %s errno %d\n",
             g_class_table[cls->class_id].path, errno);
      return -errno;
    }

  cls->fd = fd;
  btstack_run_loop_set_data_source_fd(&cls->ds, fd);
  btstack_run_loop_set_data_source_handler(&cls->ds, on_read);
  btstack_run_loop_add_data_source(&cls->ds);
  btstack_run_loop_enable_data_source_callbacks(
      &cls->ds, DATA_SOURCE_CALLBACK_READ);
  return 0;
}

static void class_close(struct class_state_s *cls)
{
  if (cls->fd < 0)
    {
      return;
    }

  btstack_run_loop_disable_data_source_callbacks(
      &cls->ds, DATA_SOURCE_CALLBACK_READ);
  btstack_run_loop_remove_data_source(&cls->ds);
  close(cls->fd);
  cls->fd = -1;

  /* Drop cached state on close so the next SENSOR ON starts unbound. */

  cls->bound       = false;
  cls->port_id     = 0xFF;
  cls->fresh       = false;
  cls->payload_len = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sensor_sampler_init(void)
{
  if (g_initialized)
    {
      return 0;
    }

  for (uint8_t i = 0; i < BTSENSOR_TLV_COUNT; i++)
    {
      class_init(&g_classes[i], i);
    }

  g_enabled     = false;
  g_initialized = true;
  return 0;
}

void sensor_sampler_deinit(void)
{
  if (!g_initialized)
    {
      return;
    }

  if (g_enabled)
    {
      sensor_sampler_set_enabled(false);
    }

  g_initialized = false;
}

int sensor_sampler_set_enabled(bool on)
{
  if (!g_initialized)
    {
      return -EINVAL;
    }

  if (on == g_enabled)
    {
      return 0;
    }

  if (on)
    {
      /* Open every class fd; failures past the first are best-effort. */

      int first_err = 0;
      for (uint8_t i = 0; i < BTSENSOR_TLV_COUNT; i++)
        {
          int rc = class_open(&g_classes[i]);
          if (rc != 0 && first_err == 0)
            {
              first_err = rc;
            }
        }

      g_enabled = true;
      syslog(LOG_INFO,
             "btsensor: SENSOR sampling on (6 class topics%s)\n",
             first_err == 0 ? "" : ", some open failed");
    }
  else
    {
      for (uint8_t i = 0; i < BTSENSOR_TLV_COUNT; i++)
        {
          class_close(&g_classes[i]);
        }

      g_enabled = false;
      syslog(LOG_INFO, "btsensor: SENSOR sampling off\n");
    }

  return 0;
}

bool sensor_sampler_is_enabled(void)
{
  return g_enabled;
}

void sensor_sampler_snapshot(
    struct sensor_class_state_s out[BTSENSOR_TLV_COUNT])
{
  uint32_t now = now_ms();

  for (uint8_t i = 0; i < BTSENSOR_TLV_COUNT; i++)
    {
      struct class_state_s *cls = &g_classes[i];
      struct sensor_class_state_s *o = &out[i];

      memset(o, 0, sizeof(*o));
      o->port_id  = 0xFF;
      o->age_10ms = BTSENSOR_TLV_AGE_SATURATED;

      if (!g_enabled)
        {
          /* SENSOR OFF: every class reported unbound regardless of cache. */

          continue;
        }

      o->port_id    = cls->port_id;
      o->mode_id    = cls->mode_id;
      o->data_type  = cls->data_type;
      o->num_values = cls->num_values;
      o->seq        = cls->seq;

      uint8_t flags = 0;
      if (cls->bound)
        {
          flags |= BTSENSOR_TLV_FLAG_BOUND;
        }

      if (cls->fresh)
        {
          flags |= BTSENSOR_TLV_FLAG_FRESH;
          o->payload_len = cls->payload_len;
          memcpy(o->payload, cls->payload,
                 cls->payload_len <= BTSENSOR_TLV_PAYLOAD_MAX
                     ? cls->payload_len : BTSENSOR_TLV_PAYLOAD_MAX);
        }

      o->flags = flags;

      /* Age in 10ms units, saturated at 0xFF (~2.55 s). */

      if (cls->last_publish_ms != 0)
        {
          uint32_t delta_ms = now - cls->last_publish_ms;
          uint32_t age10    = delta_ms / 10U;
          if (age10 > 0xFEU)
            {
              age10 = BTSENSOR_TLV_AGE_SATURATED;
            }

          o->age_10ms = (uint8_t)age10;
        }

      /* Per-call clear of fresh: each publish event surfaces FRESH=1
       * exactly once per snapshot; subsequent ticks see FRESH=0 until
       * the next publish.
       */

      cls->fresh = false;
    }
}

/****************************************************************************
 * Write APIs (Issue C)
 ****************************************************************************/

/* Common pre-flight: validates class index, SENSOR enabled, and 50 ms
 * cool-down, then opens a transient fd and takes LEGOSENSOR_CLAIM.
 * On success returns the fd (caller must close to auto-RELEASE).  On
 * failure returns -errno.
 */

static int write_open_and_claim(uint8_t class_id)
{
  if (class_id >= BTSENSOR_TLV_COUNT)
    {
      return -EINVAL;
    }

  if (!g_enabled)
    {
      return -EOPNOTSUPP;
    }

  uint32_t now = now_ms();
  uint32_t last = g_last_write_ms[class_id];
  if (last != 0 && (now - last) < SENSOR_WRITE_COOLDOWN_MS)
    {
      return -EBUSY;
    }

  int fd = open(g_class_table[class_id].path, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      return -errno;
    }

  int rc = ioctl(fd, LEGOSENSOR_CLAIM, 0);
  if (rc < 0)
    {
      int err = -errno;
      close(fd);
      return err;
    }

  return fd;
}

static void write_record_success(uint8_t class_id)
{
  g_last_write_ms[class_id] = now_ms();
}

int sensor_sampler_select_mode(uint8_t class_id, uint8_t mode)
{
  int fd = write_open_and_claim(class_id);
  if (fd < 0)
    {
      return fd;
    }

  struct legosensor_select_arg_s sel = { .mode = mode };
  int rc = ioctl(fd, LEGOSENSOR_SELECT, (unsigned long)&sel);
  if (rc < 0)
    {
      int err = -errno;
      close(fd);
      return err;
    }

  close(fd);
  write_record_success(class_id);
  return 0;
}

int sensor_sampler_send(uint8_t class_id, uint8_t mode,
                        const uint8_t *data, size_t len)
{
  if (data == NULL || len == 0 || len > BTSENSOR_TLV_PAYLOAD_MAX)
    {
      return -EINVAL;
    }

  int fd = write_open_and_claim(class_id);
  if (fd < 0)
    {
      return fd;
    }

  struct legosensor_send_arg_s snd;
  memset(&snd, 0, sizeof(snd));
  snd.mode = mode;
  snd.len  = (uint8_t)len;
  memcpy(snd.data, data, len);

  int rc = ioctl(fd, LEGOSENSOR_SEND, (unsigned long)&snd);
  if (rc < 0)
    {
      int err = -errno;
      close(fd);
      return err;
    }

  close(fd);
  write_record_success(class_id);
  return 0;
}

int sensor_sampler_set_pwm(uint8_t class_id,
                           const int16_t *channels, size_t n)
{
  if (class_id >= BTSENSOR_TLV_COUNT || channels == NULL)
    {
      return -EINVAL;
    }

  uint8_t expected = g_pwm_channels[class_id];
  if (expected == 0)
    {
      return -ENOTSUP;
    }

  if (n != expected || n > 4)
    {
      return -EINVAL;
    }

  int fd = write_open_and_claim(class_id);
  if (fd < 0)
    {
      return fd;
    }

  struct legosensor_pwm_arg_s pwm;
  memset(&pwm, 0, sizeof(pwm));
  pwm.num_channels = expected;
  for (size_t i = 0; i < n; i++)
    {
      pwm.channels[i] = channels[i];
    }

  int rc = ioctl(fd, LEGOSENSOR_SET_PWM, (unsigned long)&pwm);
  if (rc < 0)
    {
      int err = -errno;
      close(fd);
      return err;
    }

  close(fd);
  write_record_success(class_id);
  return 0;
}
