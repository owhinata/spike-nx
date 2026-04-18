/****************************************************************************
 * boards/stm32f413-discovery/scripts/memory.ld
 *
 * STM32F413ZH, BUILD_PROTECTED layout.
 *
 * Flash (1.5 MB @ 0x08000000, no reserved bootloader region):
 *
 *   kflash: 0x08000000..0x08080000  = 512 KB  (privileged kernel code)
 *   uflash: 0x08080000..0x08100000  = 512 KB  (user-space code, 2^n aligned)
 *   xflash: 0x08100000..0x08180000  = 512 KB  (user-space spare)
 *
 * SRAM (320 KB @ 0x20000000):
 *
 *   ksram:  0x20000000..0x20010000  =  64 KB  (kernel .data/.bss, linker only)
 *   usram:  0x20010000..0x20020000  =  64 KB  (user .data/.bss, 2^n aligned,
 *                                              linker only)
 *   xsram:  0x20020000..0x20050000  = 192 KB  (kernel+user heap at runtime;
 *                                              stm32_allocateheap.c carves
 *                                              the 2^n aligned user heap from
 *                                              the top and gives the rest to
 *                                              the kernel heap)
 *
 * SRAM1_END (for stm32_allocateheap.c) is 0x20050000.
 * CONFIG_NUTTX_USERSPACE must equal the start of uflash (0x08080000).
 *
 ****************************************************************************/

MEMORY
{
  kflash (rx)      : ORIGIN = 0x08000000, LENGTH = 512K
  uflash (rx)      : ORIGIN = 0x08080000, LENGTH = 512K
  xflash (rx)      : ORIGIN = 0x08100000, LENGTH = 512K

  ksram (rwx)      : ORIGIN = 0x20000000, LENGTH = 64K
  usram (rwx)      : ORIGIN = 0x20010000, LENGTH = 64K
  xsram (rwx)      : ORIGIN = 0x20020000, LENGTH = 192K
}
