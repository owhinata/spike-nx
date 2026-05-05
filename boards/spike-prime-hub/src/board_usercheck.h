/****************************************************************************
 * boards/spike-prime-hub/src/board_usercheck.h
 *
 * User pointer range validation for kernel-side char drivers under
 * CONFIG_BUILD_PROTECTED.  See Issue #41 for context.
 *
 * Why this exists:
 *   On STM32F413 (Cortex-M4 + ARM_MPU) the kernel runs in privileged mode
 *   and the MPU lets privileged code read/write user RAM directly, so a
 *   bare memcpy(kbuf, ubuf, n) inside an ioctl/write handler "works" but
 *   does not check that ubuf actually points into the calling task's
 *   user-space.  A malicious user could pass a kernel address and have
 *   the driver leak/corrupt kernel memory.  ARMv7-M / Cortex-M has no
 *   generic up_addrenv_user_vaddr() (those exist only on MMU archs), so
 *   each board driver has to validate the range itself.
 *
 *   The user-accessible memory layout for this board is fixed at link
 *   time (see boards/spike-prime-hub/scripts/memory.ld):
 *     usram   0x20020000..0x20040000  (128 KB, .data/.bss + heap head)
 *     xsram   0x20040000..0x20050000  ( 64 KB, heap tail)
 *     uflash  0x08080000..0x08100000  (512 KB, user .text/.rodata)
 *     xflash  0x08100000..0x08180000  (512 KB, reserved for user)
 *
 *   board_user_in_ok(p, n)  - p..p+n is in user-readable memory
 *                             (RAM data/heap OR user flash text/rodata).
 *                             Use for source buffers in write()/ioctl IN.
 *   board_user_out_ok(p, n) - p..p+n is in user-writable memory
 *                             (RAM data/heap only; flash is read-only).
 *                             Use for destination buffers in read()/ioctl OUT.
 *
 * Under CONFIG_BUILD_FLAT both helpers degenerate to "true" since there
 * is no privilege boundary to police.
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_SRC_BOARD_USERCHECK_H
#define __BOARDS_SPIKE_PRIME_HUB_SRC_BOARD_USERCHECK_H

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef CONFIG_BUILD_PROTECTED

#define BOARD_USRAM_START   0x20020000UL
#define BOARD_USRAM_END     0x20050000UL  /* usram (128K) + xsram (64K) */

#define BOARD_UFLASH_START  0x08080000UL
#define BOARD_UFLASH_END    0x08180000UL  /* uflash (512K) + xflash (512K) */

static inline bool board_user_out_ok(FAR const void *p, size_t n)
{
  uintptr_t a = (uintptr_t)p;
  uintptr_t b;

  if (n == 0)
    {
      return true;
    }

  if (__builtin_add_overflow(a, n, &b))
    {
      return false;
    }

  return a >= BOARD_USRAM_START && b <= BOARD_USRAM_END;
}

static inline bool board_user_in_ok(FAR const void *p, size_t n)
{
  uintptr_t a = (uintptr_t)p;
  uintptr_t b;

  if (n == 0)
    {
      return true;
    }

  if (__builtin_add_overflow(a, n, &b))
    {
      return false;
    }

  if (a >= BOARD_USRAM_START && b <= BOARD_USRAM_END)
    {
      return true;
    }

  if (a >= BOARD_UFLASH_START && b <= BOARD_UFLASH_END)
    {
      return true;
    }

  return false;
}

#else /* CONFIG_BUILD_FLAT — no privilege boundary */

static inline bool board_user_in_ok(FAR const void *p, size_t n)
{
  (void)p; (void)n;
  return true;
}

static inline bool board_user_out_ok(FAR const void *p, size_t n)
{
  (void)p; (void)n;
  return true;
}

#endif /* CONFIG_BUILD_PROTECTED */

#endif /* __BOARDS_SPIKE_PRIME_HUB_SRC_BOARD_USERCHECK_H */
