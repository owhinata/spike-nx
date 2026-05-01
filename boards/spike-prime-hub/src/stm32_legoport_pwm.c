/****************************************************************************
 * boards/spike-prime-hub/src/stm32_legoport_pwm.c
 *
 * Passive H-bridge PWM HAL for the SPIKE Prime Hub I/O ports A-F (Issue
 * #80).  Drives the 6 LEGO Powered Up DC-output ports through TIM1 / TIM3
 * / TIM4 PWM pins paired with two GPIO direction pins per port.  Port-
 * agnostic: the public API is indexed 0..5 only, mirroring the pybricks
 * `pbdrv_motor_driver_*` design (`pybricks/lib/pbio/drv/motor_driver/
 * motor_driver_hbridge_pwm.c`).
 *
 * State machine (verbatim port of pybricks):
 *   COAST          : pin1 = GPIO LOW, pin2 = GPIO LOW   (motor floats)
 *   BRAKE          : pin1 = GPIO HIGH, pin2 = GPIO HIGH (high-side short)
 *   FWD (duty>0)   : pin1 = AF (inverted PWM), pin2 = GPIO HIGH
 *                    CCR(pin1) = duty/10
 *                    -> pin1 LOW for "duty/10 ticks" each PWM period
 *                       (drive phase: pin1=LOW, pin2=HIGH)
 *                    -> pin1 HIGH for the rest (brake phase: both HIGH)
 *   REV (duty<0)   : pin1 = GPIO HIGH, pin2 = AF (inverted PWM)
 *                    CCR(pin2) = -duty/10
 *
 * Polarity: CCxP = 1 (inverted, active-low) — pybricks `platform.c:562-587`
 * sets PBDRV_PWM_STM32_TIM_CHANNEL_*_INVERT on every motor channel, which
 * `pwm_stm32_tim.c:63-64` translates to `CCER.CCxP`.  This makes "CCR
 * value = drive ticks" intuitive (duty=10000 -> CCR=1000 -> nearly 100%
 * drive).
 *
 * State-transition order (mutex-protected):
 *   1. Write the new CCR (preload, takes effect at next overflow).
 *   2. Move the "stop side" pin to its safe GPIO state first.
 *   3. Move the "drive side" pin into AF mode (or to its GPIO state).
 *   4. Update the cached state.
 *
 * REV<->FWD must transit through BRAKE/COAST so the two AF pins never
 * overlap.  pybricks does this by exposing four independent functions
 * (run_fwd / run_rev / brake / coast) — we mirror that internally.
 *
 * Clock: TIM1 on APB2 (96 MHz) and TIM3/TIM4 on APB1 with x2 prescaler
 * (96 MHz).  PSC=7 -> /8 -> 12 MHz tick.  ARR=1000 -> period 1001 ticks
 * -> ~11.99 kHz PWM (pybricks-equivalent).
 *
 * License: state machine and per-channel register layout are derived from
 * pybricks (MIT, Copyright (c) 2018-2025 The Pybricks Authors).
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mutex.h>

#include <arch/board/board_legoport.h>

#include "arm_internal.h"
#include "stm32.h"
#include "hardware/stm32_tim.h"
#include "hardware/stm32f40xxx_rcc.h"

#ifdef CONFIG_LEGO_PORT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per pybricks platform.c: 96 MHz TIM clock / 8 / 1001 ~= 11.99 kHz. */

#define LP_PWM_PSC    (7)        /* TIM PSC register: divide by 8 */
#define LP_PWM_ARR    (1000)     /* counts 0..1000 inclusive = 1001 ticks */

/* Same GPIO config helpers used by stm32_legoport.c — duplicated here so
 * the two drivers stay decoupled at source level.
 */

#define LP_OUT_HI_S(port, pin)                                              \
  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_SET   | GPIO_SPEED_50MHz |     \
   (port) | (pin))

#define LP_OUT_LO_S(port, pin)                                              \
  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_CLEAR | GPIO_SPEED_50MHz |     \
   (port) | (pin))

#define LP_AF(af, port, pin)                                                \
  (GPIO_ALT | GPIO_PUSHPULL | (af) | GPIO_SPEED_50MHz | (port) | (pin))

/* PWM mode 1 OCxM value (0b110 — RM0430 §17.4.9 / §18.4.7). */

#define OCM_PWM_MODE1 (6)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Per-port platform data.  Mirrors pybricks
 * `pbdrv_motor_driver_hbridge_pwm_platform_data` from
 * `lib/pbio/platform/prime_hub/platform.c:415-510`.
 */

struct legoport_pwm_pdata_s
{
  uint32_t  pin1_af;          /* stm32_configgpio AF mode */
  uint32_t  pin1_lo;          /* stm32_configgpio output LOW */
  uint32_t  pin1_hi;          /* stm32_configgpio output HIGH */
  uint32_t  pin2_af;
  uint32_t  pin2_lo;
  uint32_t  pin2_hi;
  uint32_t  tim_base;         /* STM32_TIM1_BASE / STM32_TIM3_BASE / STM32_TIM4_BASE */
  uint8_t   pin1_ch;          /* 1..4 */
  uint8_t   pin2_ch;          /* 1..4 */
  bool      is_advanced;      /* TIM1 only — needs BDTR.MOE */
};

struct legoport_pwm_dev_s
{
  int16_t   duty;             /* current duty (-10000..10000) */
  uint8_t   state;            /* legoport_pwm_state_e */
  bool      initialized;
  bool      pinned;           /* held by LUMP supply rail (true while
                               * NEEDS_SUPPLY_PIN1/PIN2 device is SYNCED) */
  mutex_t   lock;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Per-port platform data (verbatim from pybricks platform.c:415-495).
 * Order matches Issue #80 "idx -> Port" mapping (A=0, B=1, ..., F=5).
 */

static const struct legoport_pwm_pdata_s g_pdata[BOARD_LEGOPORT_COUNT] =
{
  /* Port A: PE9 (TIM1 CH1) / PE11 (TIM1 CH2), AF1 */
  {
    .pin1_af      = LP_AF(GPIO_AF1, GPIO_PORTE, GPIO_PIN9),
    .pin1_lo      = LP_OUT_LO_S(GPIO_PORTE, GPIO_PIN9),
    .pin1_hi      = LP_OUT_HI_S(GPIO_PORTE, GPIO_PIN9),
    .pin2_af      = LP_AF(GPIO_AF1, GPIO_PORTE, GPIO_PIN11),
    .pin2_lo      = LP_OUT_LO_S(GPIO_PORTE, GPIO_PIN11),
    .pin2_hi      = LP_OUT_HI_S(GPIO_PORTE, GPIO_PIN11),
    .tim_base     = STM32_TIM1_BASE,
    .pin1_ch      = 1,
    .pin2_ch      = 2,
    .is_advanced  = true,
  },

  /* Port B: PE13 (TIM1 CH3) / PE14 (TIM1 CH4), AF1 */
  {
    .pin1_af      = LP_AF(GPIO_AF1, GPIO_PORTE, GPIO_PIN13),
    .pin1_lo      = LP_OUT_LO_S(GPIO_PORTE, GPIO_PIN13),
    .pin1_hi      = LP_OUT_HI_S(GPIO_PORTE, GPIO_PIN13),
    .pin2_af      = LP_AF(GPIO_AF1, GPIO_PORTE, GPIO_PIN14),
    .pin2_lo      = LP_OUT_LO_S(GPIO_PORTE, GPIO_PIN14),
    .pin2_hi      = LP_OUT_HI_S(GPIO_PORTE, GPIO_PIN14),
    .tim_base     = STM32_TIM1_BASE,
    .pin1_ch      = 3,
    .pin2_ch      = 4,
    .is_advanced  = true,
  },

  /* Port C: PB6 (TIM4 CH1) / PB7 (TIM4 CH2), AF2 */
  {
    .pin1_af      = LP_AF(GPIO_AF2, GPIO_PORTB, GPIO_PIN6),
    .pin1_lo      = LP_OUT_LO_S(GPIO_PORTB, GPIO_PIN6),
    .pin1_hi      = LP_OUT_HI_S(GPIO_PORTB, GPIO_PIN6),
    .pin2_af      = LP_AF(GPIO_AF2, GPIO_PORTB, GPIO_PIN7),
    .pin2_lo      = LP_OUT_LO_S(GPIO_PORTB, GPIO_PIN7),
    .pin2_hi      = LP_OUT_HI_S(GPIO_PORTB, GPIO_PIN7),
    .tim_base     = STM32_TIM4_BASE,
    .pin1_ch      = 1,
    .pin2_ch      = 2,
    .is_advanced  = false,
  },

  /* Port D: PB8 (TIM4 CH3) / PB9 (TIM4 CH4), AF2 */
  {
    .pin1_af      = LP_AF(GPIO_AF2, GPIO_PORTB, GPIO_PIN8),
    .pin1_lo      = LP_OUT_LO_S(GPIO_PORTB, GPIO_PIN8),
    .pin1_hi      = LP_OUT_HI_S(GPIO_PORTB, GPIO_PIN8),
    .pin2_af      = LP_AF(GPIO_AF2, GPIO_PORTB, GPIO_PIN9),
    .pin2_lo      = LP_OUT_LO_S(GPIO_PORTB, GPIO_PIN9),
    .pin2_hi      = LP_OUT_HI_S(GPIO_PORTB, GPIO_PIN9),
    .tim_base     = STM32_TIM4_BASE,
    .pin1_ch      = 3,
    .pin2_ch      = 4,
    .is_advanced  = false,
  },

  /* Port E: PC6 (TIM3 CH1) / PC7 (TIM3 CH2), AF2.  Conflicts with USART6 —
   * see board.h:117-128 and Kconfig `depends on !STM32_USART6`.
   */
  {
    .pin1_af      = LP_AF(GPIO_AF2, GPIO_PORTC, GPIO_PIN6),
    .pin1_lo      = LP_OUT_LO_S(GPIO_PORTC, GPIO_PIN6),
    .pin1_hi      = LP_OUT_HI_S(GPIO_PORTC, GPIO_PIN6),
    .pin2_af      = LP_AF(GPIO_AF2, GPIO_PORTC, GPIO_PIN7),
    .pin2_lo      = LP_OUT_LO_S(GPIO_PORTC, GPIO_PIN7),
    .pin2_hi      = LP_OUT_HI_S(GPIO_PORTC, GPIO_PIN7),
    .tim_base     = STM32_TIM3_BASE,
    .pin1_ch      = 1,
    .pin2_ch      = 2,
    .is_advanced  = false,
  },

  /* Port F: PC8 (TIM3 CH3) / PB1 (TIM3 CH4), AF2 */
  {
    .pin1_af      = LP_AF(GPIO_AF2, GPIO_PORTC, GPIO_PIN8),
    .pin1_lo      = LP_OUT_LO_S(GPIO_PORTC, GPIO_PIN8),
    .pin1_hi      = LP_OUT_HI_S(GPIO_PORTC, GPIO_PIN8),
    .pin2_af      = LP_AF(GPIO_AF2, GPIO_PORTB, GPIO_PIN1),
    .pin2_lo      = LP_OUT_LO_S(GPIO_PORTB, GPIO_PIN1),
    .pin2_hi      = LP_OUT_HI_S(GPIO_PORTB, GPIO_PIN1),
    .tim_base     = STM32_TIM3_BASE,
    .pin1_ch      = 3,
    .pin2_ch      = 4,
    .is_advanced  = false,
  },
};

static struct legoport_pwm_dev_s g_devs[BOARD_LEGOPORT_COUNT];

/****************************************************************************
 * Private Functions: TIM register helpers
 ****************************************************************************/

/* Compute the offset of CCRn within a TIM register block.
 * Channel layout (advanced and general timers are identical for CCR1..4):
 *   CCR1 = +0x34, CCR2 = +0x38, CCR3 = +0x3C, CCR4 = +0x40
 */

static uintptr_t tim_ccr_addr(uintptr_t tim_base, uint8_t ch)
{
  /* Use the official offsets from stm32_tim_v1v2.h.  They happen to match
   * for both ATIM (TIM1/8) and GTIM (TIM2-5).
   */

  switch (ch)
    {
      case 1: return tim_base + STM32_GTIM_CCR1_OFFSET;
      case 2: return tim_base + STM32_GTIM_CCR2_OFFSET;
      case 3: return tim_base + STM32_GTIM_CCR3_OFFSET;
      case 4: return tim_base + STM32_GTIM_CCR4_OFFSET;
      default: return 0;
    }
}

/* Configure CCMR for a given channel: PWM mode 1 + OCxPE preload.
 * CCMR1 holds CH1/CH2, CCMR2 holds CH3/CH4 (both ATIM and GTIM).
 *   CH1: bits 4-6 OC1M, bit 3 OC1PE
 *   CH2: bits 12-14 OC2M, bit 11 OC2PE
 *   CH3: bits 4-6 OC3M, bit 3 OC3PE
 *   CH4: bits 12-14 OC4M, bit 11 OC4PE
 */

static void tim_configure_channel_ccmr(uintptr_t tim_base, uint8_t ch)
{
  uintptr_t ccmr;
  uint32_t  shift_ocm;
  uint32_t  shift_ocpe;
  uint32_t  reg;

  if (ch == 1 || ch == 2)
    {
      ccmr = tim_base + STM32_GTIM_CCMR1_OFFSET;
    }
  else
    {
      ccmr = tim_base + STM32_GTIM_CCMR2_OFFSET;
    }

  if (ch == 1 || ch == 3)
    {
      shift_ocm  = 4;     /* OC1M / OC3M */
      shift_ocpe = 3;     /* OC1PE / OC3PE */
    }
  else
    {
      shift_ocm  = 12;    /* OC2M / OC4M */
      shift_ocpe = 11;    /* OC2PE / OC4PE */
    }

  reg = getreg32(ccmr);

  /* Clear any prior mode bits + clear preload bit. */

  reg &= ~((uint32_t)(7 << shift_ocm));
  reg &= ~((uint32_t)(1 << shift_ocpe));

  /* PWM mode 1 + preload. */

  reg |= (OCM_PWM_MODE1 << shift_ocm);
  reg |= (1u << shift_ocpe);

  putreg32(reg, ccmr);
}

/* Set CCER bits for a channel: CCxE=1, CCxP=1 (inverted), CCxNE=0. */

static void tim_configure_channel_ccer(uintptr_t tim_base, uint8_t ch)
{
  uintptr_t ccer = tim_base + STM32_GTIM_CCER_OFFSET;
  uint32_t  reg  = getreg32(ccer);
  uint32_t  base_shift = (uint32_t)((ch - 1) * 4);

  /* Clear the 4 bits for this channel: CCxE / CCxP / CCxNE / CCxNP. */

  reg &= ~((uint32_t)(0xF << base_shift));

  /* Set CCxE (bit 0 of the 4-bit group) and CCxP (bit 1). */

  reg |= (1u << (base_shift + 0));   /* CCxE = 1 */
  reg |= (1u << (base_shift + 1));   /* CCxP = 1 (inverted) */

  putreg32(reg, ccer);
}

/* RCC clock enable for a TIM block.  Per RM0430 §6.3 (RCC):
 *   TIM1 -> RCC_APB2ENR.TIM1EN  (bit 0)
 *   TIM3 -> RCC_APB1ENR.TIM3EN  (bit 1)
 *   TIM4 -> RCC_APB1ENR.TIM4EN  (bit 2)
 */

static void tim_rcc_enable(uintptr_t tim_base)
{
  if (tim_base == STM32_TIM1_BASE)
    {
      modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM1EN);
    }
  else if (tim_base == STM32_TIM3_BASE)
    {
      modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM3EN);
    }
  else if (tim_base == STM32_TIM4_BASE)
    {
      modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM4EN);
    }
}

/* One-shot init for a single TIM block.  Idempotent — calling twice for
 * the same TIM (because two ports share a TIM) is intentional and safe.
 */

static void tim_block_init(uintptr_t tim_base, bool is_advanced)
{
  /* 1. Enable the clock so register writes don't fault. */

  tim_rcc_enable(tim_base);

  /* 2. Stop the timer while we reconfigure. */

  modifyreg32(tim_base + STM32_GTIM_CR1_OFFSET, GTIM_CR1_CEN, 0);

  /* 3. Disable any update / DMA / break IRQ. */

  putreg32(0, tim_base + STM32_GTIM_DIER_OFFSET);

  /* 4. PSC and ARR. */

  putreg32(LP_PWM_PSC, tim_base + STM32_GTIM_PSC_OFFSET);
  putreg32(LP_PWM_ARR, tim_base + STM32_GTIM_ARR_OFFSET);

  /* 5. ARR preload + counter mode = up. */

  modifyreg32(tim_base + STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_ARPE);

  /* 6. CCRx = 0 for all 4 channels (start at 0% drive). */

  putreg32(0, tim_base + STM32_GTIM_CCR1_OFFSET);
  putreg32(0, tim_base + STM32_GTIM_CCR2_OFFSET);
  putreg32(0, tim_base + STM32_GTIM_CCR3_OFFSET);
  putreg32(0, tim_base + STM32_GTIM_CCR4_OFFSET);

  /* 7. CCMR for all 4 channels — PWM mode 1 + preload.  Even though only
   *    2 channels are wired up per port, both ports of a shared TIM go
   *    through here so we initialize CH1..4 unconditionally.
   */

  tim_configure_channel_ccmr(tim_base, 1);
  tim_configure_channel_ccmr(tim_base, 2);
  tim_configure_channel_ccmr(tim_base, 3);
  tim_configure_channel_ccmr(tim_base, 4);

  /* 8. CCER for all 4 channels — CCxE=1, CCxP=1 (inverted). */

  tim_configure_channel_ccer(tim_base, 1);
  tim_configure_channel_ccer(tim_base, 2);
  tim_configure_channel_ccer(tim_base, 3);
  tim_configure_channel_ccer(tim_base, 4);

  /* 9. TIM1 advanced timer: BDTR.MOE = 1 enables OCx output gating.
   *    Without this, OCx pins remain High-Z regardless of CCxE.
   *    Other BDTR bits (BKE, OSSI, OSSR) are left at reset 0 (no break
   *    input, off-state OE disabled).
   */

  if (is_advanced)
    {
      putreg32(ATIM_BDTR_MOE,
               tim_base + STM32_ATIM_BDTR_OFFSET);
    }

  /* 10. Force shadow registers to load (UG event). */

  modifyreg32(tim_base + STM32_GTIM_EGR_OFFSET, 0, GTIM_EGR_UG);

  /* 11. Start the timer. */

  modifyreg32(tim_base + STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_CEN);
}

/****************************************************************************
 * Private Functions: state transitions
 ****************************************************************************/

static void hbridge_apply_coast(int idx)
{
  const struct legoport_pwm_pdata_s *pd = &g_pdata[idx];

  /* Both pins LOW.  Order doesn't matter — neither pin is in AF, no
   * shoot-through risk.
   */

  stm32_configgpio(pd->pin1_lo);
  stm32_configgpio(pd->pin2_lo);

  g_devs[idx].state = LEGOPORT_PWM_STATE_COAST;
  g_devs[idx].duty  = 0;
}

static void hbridge_apply_brake(int idx)
{
  const struct legoport_pwm_pdata_s *pd = &g_pdata[idx];

  /* Both pins HIGH.  If we were previously in FWD/REV one pin is in AF
   * mode — switch it to GPIO output HIGH first, then the other.
   */

  stm32_configgpio(pd->pin1_hi);
  stm32_configgpio(pd->pin2_hi);

  g_devs[idx].state = LEGOPORT_PWM_STATE_BRAKE;
  g_devs[idx].duty  = 0;
}

static void hbridge_apply_fwd(int idx, int16_t duty)
{
  const struct legoport_pwm_pdata_s *pd = &g_pdata[idx];
  uintptr_t ccr = tim_ccr_addr(pd->tim_base, pd->pin1_ch);

  /* 1. Write CCR first (preload — applied at next overflow). */

  putreg32((uint32_t)(duty / 10), ccr);

  /* 2. "Stop side" pin2 -> output HIGH first (if it was AF, this
   *    immediately brings it out of AF mode without going through input).
   */

  stm32_configgpio(pd->pin2_hi);

  /* 3. "Drive side" pin1 -> AF (PWM).  pin1_af enables ALT mode, and
   *    because CCER.CCxP=1 the pin idles HIGH and pulses LOW for CCR
   *    ticks per period (= drive phase).
   */

  stm32_configgpio(pd->pin1_af);

  g_devs[idx].state = LEGOPORT_PWM_STATE_PWM;
  g_devs[idx].duty  = duty;
}

static void hbridge_apply_rev(int idx, int16_t duty)
{
  const struct legoport_pwm_pdata_s *pd = &g_pdata[idx];
  uintptr_t ccr = tim_ccr_addr(pd->tim_base, pd->pin2_ch);

  putreg32((uint32_t)((-duty) / 10), ccr);

  /* "Stop side" is pin1 in REV. */

  stm32_configgpio(pd->pin1_hi);
  stm32_configgpio(pd->pin2_af);

  g_devs[idx].state = LEGOPORT_PWM_STATE_PWM;
  g_devs[idx].duty  = duty;
}

/* Generic "go via brake" trampoline used when the current state has one
 * pin in AF and the new state needs the other pin in AF (REV<->FWD), or
 * when entering PWM mode from COAST.  Honors pybricks' rule of routing
 * every transition through a fully-GPIO state.
 */

static void hbridge_set_duty_internal(int idx, int16_t duty)
{
  uint8_t prev_state = g_devs[idx].state;
  int16_t prev_duty  = g_devs[idx].duty;

  /* duty = 0 (or sign change) goes through BRAKE first per pybricks. */

  if (duty == 0)
    {
      hbridge_apply_brake(idx);
      return;
    }

  /* If we're already in PWM with the same sign, only the CCR needs
   * updating (no GPIO change).  This is the steady-state hot path.
   */

  if (prev_state == LEGOPORT_PWM_STATE_PWM &&
      ((prev_duty > 0 && duty > 0) || (prev_duty < 0 && duty < 0)))
    {
      const struct legoport_pwm_pdata_s *pd = &g_pdata[idx];
      uint8_t ch = (duty > 0) ? pd->pin1_ch : pd->pin2_ch;
      int16_t mag = (duty > 0) ? duty : (int16_t)-duty;
      putreg32((uint32_t)(mag / 10), tim_ccr_addr(pd->tim_base, ch));
      g_devs[idx].duty = duty;
      return;
    }

  /* Otherwise transit through BRAKE so the AF assignment can be flipped
   * without a moment where both pins are simultaneously in AF.
   */

  hbridge_apply_brake(idx);

  if (duty > 0)
    {
      hbridge_apply_fwd(idx, duty);
    }
  else
    {
      hbridge_apply_rev(idx, duty);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_legoport_pwm_initialize(void)
{
  int p;

  /* TIM block init — three blocks for six ports. */

  tim_block_init(STM32_TIM1_BASE, true);
  tim_block_init(STM32_TIM3_BASE, false);
  tim_block_init(STM32_TIM4_BASE, false);

  /* Per-port HAL state + start in COAST so motors don't move. */

  for (p = 0; p < BOARD_LEGOPORT_COUNT; p++)
    {
      memset(&g_devs[p], 0, sizeof(g_devs[p]));
      nxmutex_init(&g_devs[p].lock);
      g_devs[p].state       = LEGOPORT_PWM_STATE_COAST;
      g_devs[p].duty        = 0;
      g_devs[p].initialized = true;

      hbridge_apply_coast(p);
    }

  return OK;
}

int stm32_legoport_pwm_set_duty(int idx, int16_t duty)
{
  int ret;

  if (idx < 0 || idx >= BOARD_LEGOPORT_COUNT)
    {
      return -EINVAL;
    }

  if (duty < LEGOPORT_PWM_DUTY_MIN || duty > LEGOPORT_PWM_DUTY_MAX)
    {
      return -ERANGE;
    }

  if (!g_devs[idx].initialized)
    {
      return -ENODEV;
    }

  ret = nxmutex_lock(&g_devs[idx].lock);
  if (ret < 0)
    {
      return ret;
    }

  /* While LUMP holds the H-bridge as a SUPPLY rail (Color / Ultrasonic
   * sensor that announces NEEDS_SUPPLY_PIN1/PIN2), userspace must not
   * disturb the duty.  Mirrors how pybricks user-level motor APIs
   * cannot reach a non-motor LUMP device through the type category
   * gate in `legodev_spec_device_category_match()`.
   */

  if (g_devs[idx].pinned)
    {
      nxmutex_unlock(&g_devs[idx].lock);
      return -EBUSY;
    }

  hbridge_set_duty_internal(idx, duty);
  nxmutex_unlock(&g_devs[idx].lock);
  return OK;
}

int stm32_legoport_pwm_coast(int idx)
{
  int ret;

  if (idx < 0 || idx >= BOARD_LEGOPORT_COUNT)
    {
      return -EINVAL;
    }

  if (!g_devs[idx].initialized)
    {
      return -ENODEV;
    }

  ret = nxmutex_lock(&g_devs[idx].lock);
  if (ret < 0)
    {
      return ret;
    }

  /* close()-time auto-COAST also reaches us — stay silent (no -EBUSY)
   * for the pinned case so userspace closing a fd does not see an
   * error code, but do not actually drop the supply rail.
   */

  if (g_devs[idx].pinned)
    {
      nxmutex_unlock(&g_devs[idx].lock);
      return OK;
    }

  hbridge_apply_coast(idx);
  nxmutex_unlock(&g_devs[idx].lock);
  return OK;
}

int stm32_legoport_pwm_brake(int idx)
{
  int ret;

  if (idx < 0 || idx >= BOARD_LEGOPORT_COUNT)
    {
      return -EINVAL;
    }

  if (!g_devs[idx].initialized)
    {
      return -ENODEV;
    }

  ret = nxmutex_lock(&g_devs[idx].lock);
  if (ret < 0)
    {
      return ret;
    }

  if (g_devs[idx].pinned)
    {
      nxmutex_unlock(&g_devs[idx].lock);
      return -EBUSY;
    }

  hbridge_apply_brake(idx);
  nxmutex_unlock(&g_devs[idx].lock);
  return OK;
}

/* LUMP-internal API: pin a port as a SUPPLY rail.  Called from the
 * LUMP engine (`stm32_legoport_lump.c`) right after SYNC reaches
 * SYNCED on a device that announced NEEDS_SUPPLY_PIN1/PIN2.  Mirrors
 * the pybricks `legodev_pup_uart.c:894-897` step that calls
 * `pbdrv_motor_driver_set_duty_cycle(driver, ±MAX_DUTY)` straight
 * after the baud-rate switch.
 *
 *   sign > 0 -> NEEDS_SUPPLY_PIN2 (FWD full drive, pin2=PWM-LOW)
 *   sign < 0 -> NEEDS_SUPPLY_PIN1 (REV full drive, pin1=PWM-LOW)
 *   sign = 0 -> coast (-EINVAL, since pinning to coast is meaningless)
 */

int stm32_legoport_pwm_pin_supply(int idx, int sign)
{
  int ret;

  if (idx < 0 || idx >= BOARD_LEGOPORT_COUNT || sign == 0)
    {
      return -EINVAL;
    }

  if (!g_devs[idx].initialized)
    {
      return -ENODEV;
    }

  ret = nxmutex_lock(&g_devs[idx].lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Drive the H-bridge to ±MAX_DUTY before flipping the pinned flag —
   * after pinned=true, set_duty_internal cannot reach this idx so the
   * configuration must be in place first.
   */

  hbridge_set_duty_internal(idx,
                            sign > 0 ? LEGOPORT_PWM_DUTY_MAX
                                     : LEGOPORT_PWM_DUTY_MIN);
  g_devs[idx].pinned = true;

  nxmutex_unlock(&g_devs[idx].lock);
  return OK;
}

/* LUMP-internal API: drop the supply pin and coast.  Called from
 * the LUMP engine reset path (disconnect / ERR / re-init) and from
 * `stm32_legoport_pwm_initialize()` semantics if a re-init ever
 * happens at runtime.
 */

int stm32_legoport_pwm_unpin(int idx)
{
  int ret;

  if (idx < 0 || idx >= BOARD_LEGOPORT_COUNT)
    {
      return -EINVAL;
    }

  if (!g_devs[idx].initialized)
    {
      return -ENODEV;
    }

  ret = nxmutex_lock(&g_devs[idx].lock);
  if (ret < 0)
    {
      return ret;
    }

  g_devs[idx].pinned = false;
  hbridge_apply_coast(idx);

  nxmutex_unlock(&g_devs[idx].lock);
  return OK;
}

int stm32_legoport_pwm_get_status(int idx,
                                  FAR struct legoport_pwm_status_s *out)
{
  int ret;

  if (idx < 0 || idx >= BOARD_LEGOPORT_COUNT)
    {
      return -EINVAL;
    }

  if (out == NULL)
    {
      return -EINVAL;
    }

  if (!g_devs[idx].initialized)
    {
      return -ENODEV;
    }

  ret = nxmutex_lock(&g_devs[idx].lock);
  if (ret < 0)
    {
      return ret;
    }

  memset(out, 0, sizeof(*out));
  out->duty  = g_devs[idx].duty;
  out->state = g_devs[idx].state;
  if (g_devs[idx].pinned)
    {
      out->flags |= LEGOPORT_PWM_FLAG_PINNED;
    }

  nxmutex_unlock(&g_devs[idx].lock);
  return OK;
}

#endif /* CONFIG_LEGO_PORT */
