/****************************************************************************
 * boards/spike-prime-hub/src/stm32_bcrumb.c
 *
 * Reset reason + breadcrumb capture across MCU reset.
 *
 * Carves the last 64 bytes of the kernel heap (0x2001FFC0..0x20020000) as a
 * non-cleared "breadcrumb" struct that survives any MCU reset where Vdd
 * holds (NVIC SystemReset, IWDG, software-induced reset).  At every boot,
 * stm32_bcrumb_initialize():
 *
 *   1. Reads RCC_CSR to identify which HW reset flag fired.
 *   2. If the previous boot left a valid magic, prints the recorded
 *      pre-reset reason and breadcrumb words to syslog (RAMLOG so the
 *      message survives even when USB CDC is briefly absent).
 *   3. Clears RCC_CSR reset flags and re-initializes the struct with a
 *      fresh magic so the next reset cycle starts from a known state.
 *
 * Producers:
 *   - board_reset(status)           -> stm32_bcrumb_set_board_reset(status)
 *   - hpwork_softdog_check PANIC()  -> stm32_bcrumb_set_pre_reason(SOFTDOG, age)
 *   - HardFault / assert path is naturally covered via board_reset() because
 *     CONFIG_BOARD_RESET_ON_ASSERT=2 routes asserts through board_reset.
 *
 * Memory layout note: kernel heap is g_idle_topstack..SPIKE_USRAM_BASE.
 * up_allocate_kheap() in stm32_allocateheap.c reserves the top 64 bytes
 * for this struct so the heap allocator never overwrites it.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/arch.h>

#include "arm_internal.h"
#include "stm32.h"

#include "spike_prime_hub.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BCRUMB_BASE     0x2001FFC0u   /* top 64 bytes of kernel heap */
#define BCRUMB_MAGIC    0x42435242u   /* 'BCRB' */

struct bcrumb_s
{
  uint32_t magic;                 /* BCRUMB_MAGIC when valid */
  uint32_t pre_reason;            /* BCRUMB_PRE_* — first cause to be set */
  uint32_t pre_reason_arg;        /* domain-specific data (softdog age_ms etc) */
  uint32_t board_reset_status;    /* status arg of last board_reset() */
  uint32_t rcc_csr_at_init;       /* RCC_CSR captured at *next* boot */
  uint32_t marker[10];             /* free-form breadcrumb words */
  uint32_t reserved;
};                                /* 16 words = 64 bytes */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile struct bcrumb_s * const g_bcrumb =
    (volatile struct bcrumb_s *)BCRUMB_BASE;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *pre_reason_name(uint32_t r)
{
  switch (r)
    {
      case BCRUMB_PRE_NONE:           return "NONE";
      case BCRUMB_PRE_SOFTDOG:        return "SOFTDOG";
      case BCRUMB_PRE_USER_REBOOT:    return "USER_REBOOT";
      case BCRUMB_PRE_POWER_BUTTON:   return "POWER_BUTTON";
      case BCRUMB_PRE_ASSERT_HOOK:    return "ASSERT_HOOK";
      default:                        return "?";
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Producer hooks — keep extremely small (called from PANIC paths and IRQ).
 * Set the *first* cause only so a downstream board_reset() does not
 * overwrite the original reason.
 */

void stm32_bcrumb_set_pre_reason(uint32_t reason, uint32_t arg)
{
  if (g_bcrumb->magic == BCRUMB_MAGIC && g_bcrumb->pre_reason == BCRUMB_PRE_NONE)
    {
      g_bcrumb->pre_reason     = reason;
      g_bcrumb->pre_reason_arg = arg;
    }
}

void stm32_bcrumb_set_board_reset(int status)
{
  if (g_bcrumb->magic == BCRUMB_MAGIC)
    {
      g_bcrumb->board_reset_status = (uint32_t)status;
    }
}

void stm32_bcrumb_set_marker(unsigned int slot, uint32_t value)
{
  if (g_bcrumb->magic == BCRUMB_MAGIC && slot < 10u)
    {
      g_bcrumb->marker[slot] = value;
    }
}

/* Per-worker entry / exit breadcrumb.  HPWORK is single-threaded so within
 * HPWORK there is no race; softdog reads in IRQ ctx as a stable snapshot.
 * Encoding: marker[1] = (id << 24) | (entry_counter & 0xFFFFFF), same for
 * marker[2] / exit_counter.  At reset analysis time, if marker[1] != marker[2]
 * the worker IDed in the high byte was *inside* HPWORK when the stall began.
 */

void stm32_bcrumb_worker_entry(unsigned int worker_id)
{
  if (g_bcrumb->magic != BCRUMB_MAGIC)
    {
      return;
    }

  static uint32_t s_entry_counter;
  s_entry_counter++;
  g_bcrumb->marker[1] = ((worker_id & 0xFFu) << 24) |
                        (s_entry_counter & 0x00FFFFFFu);
}

void stm32_bcrumb_worker_exit(unsigned int worker_id)
{
  if (g_bcrumb->magic != BCRUMB_MAGIC)
    {
      return;
    }

  static uint32_t s_exit_counter;
  s_exit_counter++;
  g_bcrumb->marker[2] = ((worker_id & 0xFFu) << 24) |
                        (s_exit_counter & 0x00FFFFFFu);
}

/* Boot-time read-out + reset.  Call as early as possible from stm32_bringup
 * so the BCRUMB line lands near the top of dmesg.
 */

void stm32_bcrumb_initialize(void)
{
  uint32_t rcc_csr = getreg32(STM32_RCC_CSR);
  uint32_t prev_magic = g_bcrumb->magic;

  /* Decode RCC_CSR reset flags. */

  syslog(LOG_INFO,
         "BCRUMB: RCC_CSR=0x%08lx [%s%s%s%s%s%s%s]\n",
         (unsigned long)rcc_csr,
         (rcc_csr & RCC_CSR_LPWRRSTF) ? "LPWR " : "",
         (rcc_csr & RCC_CSR_WWDGRSTF) ? "WWDG " : "",
         (rcc_csr & RCC_CSR_IWDGRSTF) ? "IWDG " : "",
         (rcc_csr & RCC_CSR_SFTRSTF)  ? "SFT "  : "",
         (rcc_csr & RCC_CSR_PORRSTF)  ? "POR "  : "",
         (rcc_csr & RCC_CSR_PINRSTF)  ? "PIN "  : "",
         (rcc_csr & RCC_CSR_BORRSTF)  ? "BOR "  : "");

  if (prev_magic == BCRUMB_MAGIC)
    {
      uint32_t pre   = g_bcrumb->pre_reason;
      uint32_t pre_a = g_bcrumb->pre_reason_arg;
      uint32_t br    = g_bcrumb->board_reset_status;

      syslog(LOG_INFO,
             "BCRUMB: prev pre_reason=%lu(%s) arg=0x%08lx, "
             "board_reset_status=%lu\n",
             (unsigned long)pre, pre_reason_name(pre),
             (unsigned long)pre_a,
             (unsigned long)br);

      syslog(LOG_INFO,
             "BCRUMB: markers="
             "[%08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx]\n",
             (unsigned long)g_bcrumb->marker[0],
             (unsigned long)g_bcrumb->marker[1],
             (unsigned long)g_bcrumb->marker[2],
             (unsigned long)g_bcrumb->marker[3],
             (unsigned long)g_bcrumb->marker[4],
             (unsigned long)g_bcrumb->marker[5],
             (unsigned long)g_bcrumb->marker[6],
             (unsigned long)g_bcrumb->marker[7],
             (unsigned long)g_bcrumb->marker[8],
             (unsigned long)g_bcrumb->marker[9]);
    }
  else
    {
      syslog(LOG_INFO,
             "BCRUMB: no prev breadcrumb (magic=0x%08lx, fresh power-on?)\n",
             (unsigned long)prev_magic);
    }

  /* Re-initialize the struct for this boot's lifetime. */

  memset((void *)g_bcrumb, 0, sizeof(*g_bcrumb));
  g_bcrumb->magic           = BCRUMB_MAGIC;
  g_bcrumb->pre_reason      = BCRUMB_PRE_NONE;
  g_bcrumb->rcc_csr_at_init = rcc_csr;

  /* Clear RCC reset flags so the *next* reset's RCC_CSR reflects only
   * the next event.
   */

  modifyreg32(STM32_RCC_CSR, 0, RCC_CSR_RMVF);
}
