/****************************************************************************
 * boards/spike-prime-hub/scripts/memory.ld
 *
 * STM32F413VGT6 SPIKE Prime Hub, BUILD_PROTECTED layout.
 *
 * Flash map (1.5 MB @ 0x08000000; 0x08000000..0x08008000 is the LEGO
 * bootloader and must not be overwritten):
 *
 *   kflash: 0x08008000..0x08080000  = 480 KB  (privileged kernel code)
 *   uflash: 0x08080000..0x08100000  = 512 KB  (user-space code, 2^n aligned)
 *   xflash: 0x08100000..0x08180000  = 512 KB  (user-space extras / future)
 *
 * SRAM map (320 KB @ 0x20000000):
 *
 *   ksram:  0x20000000..0x2000c000  =  48 KB  (kernel .data/.bss + idle stack;
 *                                              linker only)
 *   usram:  0x2000c000..0x20020000  =  80 KB  (user .data/.bss; linker only.
 *                                              MPU exposes it via a 128 KB
 *                                              region @ 0x20000000 with
 *                                              SRD = 0x07 (subregions 0..2
 *                                              disabled = ksram), computed
 *                                              automatically by
 *                                              mpu_user_intsram())
 *   xsram:  0x20020000..0x20050000  = 192 KB  (kernel+user heap at runtime;
 *                                              stm32_allocateheap.c carves
 *                                              the 2^n aligned user heap from
 *                                              the top and gives the rest to
 *                                              the kernel heap)
 *
 * The linker only places .data/.bss into ksram and usram.  The runtime
 * heap split (kernel heap below, user heap on top) is handled in
 * stm32_allocateheap.c and honours CONFIG_MM_KERNEL_HEAPSIZE as a lower
 * bound.  SRAM1_END (end of the runtime user heap) is 0x20050000.
 *
 * Sizing rationale (Issue #93): kernel .data/.bss currently ends at
 * _ebss = 0x20007448 (29 768 B) and the 2 KB idle stack lifts the high-
 * water to g_idle_topstack = 0x20007c48.  Shrinking ksram from 64 KB to
 * 48 KB therefore leaves ~17 KB of headroom and the freed 16 KB is
 * adjacent to (and below) usram, so we extend usram down to 0x2000c000
 * to grow the user .data/.bss budget from 64 KB to 80 KB.  The MPU
 * subregion logic (mpu_user_intsram -> mpu_configure_region) handles
 * the 80 KB / non-power-of-two size by disabling the lowest 3
 * subregions of a 128 KB region anchored at 0x20000000.
 *
 * CONFIG_NUTTX_USERSPACE must equal the start of uflash (0x08080000).
 *
 ****************************************************************************/

MEMORY
{
  kflash (rx)      : ORIGIN = 0x08008000, LENGTH = 480K
  uflash (rx)      : ORIGIN = 0x08080000, LENGTH = 512K
  xflash (rx)      : ORIGIN = 0x08100000, LENGTH = 512K

  ksram (rwx)      : ORIGIN = 0x20000000, LENGTH = 48K
  usram (rwx)      : ORIGIN = 0x2000c000, LENGTH = 80K
  xsram (rwx)      : ORIGIN = 0x20020000, LENGTH = 192K
}
