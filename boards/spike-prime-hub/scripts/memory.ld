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
 * SRAM map (320 KB @ 0x20000000, post-#98):
 *
 *   ksram:       0x20000000..0x20010000  =  64 KB  (kernel .data/.bss + idle
 *                                                   stack at the bottom; the
 *                                                   tail above g_idle_topstack
 *                                                   is reused as kernel heap)
 *   kheap slot:  0x20010000..0x20020000  =  64 KB  (runtime, NOT in MEMORY;
 *                                                   merged with the ksram
 *                                                   tail into one contiguous
 *                                                   kernel heap returned from
 *                                                   up_allocate_kheap)
 *   usram:       0x20020000..0x20040000  = 128 KB  (user .data/.bss; the
 *                                                   tail past us_bssend is
 *                                                   reused as user heap.
 *                                                   2^17 aligned; full 128 KB
 *                                                   exposed via MPU as a
 *                                                   single user RW region)
 *   xsram:       0x20040000..0x20050000  =  64 KB  (user heap top portion;
 *                                                   2^16 aligned MPU region
 *                                                   added by up_allocate_heap
 *                                                   so the user heap is a
 *                                                   single contiguous span
 *                                                   from us_bssend to
 *                                                   SRAM1_END)
 *
 * The linker only places .data/.bss into ksram and usram.  The runtime
 * heap split uses sliding boundaries on both sides:
 *   - kernel heap = g_idle_topstack..0x20020000 (~96 KB, sliding against
 *                   the idle stack top)
 *   - user heap   = us_bssend..0x20050000 (sliding boundary against bss)
 * stm32_allocateheap.c implements this; CONFIG_MM_KERNEL_HEAPSIZE is the
 * floor and is asserted at compile time to be <= 64 KB so the dedicated
 * 64 KB slot alone satisfies it even if the ksram tail were ever consumed
 * entirely by the idle stack.  SRAM1_END is 0x20050000.
 *
 * CONFIG_NUTTX_USERSPACE must equal the start of uflash (0x08080000).
 *
 ****************************************************************************/

MEMORY
{
  kflash (rx)      : ORIGIN = 0x08008000, LENGTH = 480K
  uflash (rx)      : ORIGIN = 0x08080000, LENGTH = 512K
  xflash (rx)      : ORIGIN = 0x08100000, LENGTH = 512K

  ksram (rwx)      : ORIGIN = 0x20000000, LENGTH = 64K
  usram (rwx)      : ORIGIN = 0x20020000, LENGTH = 128K
  xsram (rwx)      : ORIGIN = 0x20040000, LENGTH = 64K
}
