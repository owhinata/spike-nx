/****************************************************************************
 * boards/stm32f413-discovery/scripts/kernel-space.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/* NOTE: This depends on memory.ld being included first. */

OUTPUT_ARCH(arm)
ENTRY(_stext)
EXTERN(_vectors)
SECTIONS
{
    .text : {
        _stext = ABSOLUTE(.);
        *(.vectors)
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } > kflash

    .init_section : ALIGN(4) {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > kflash

    .ARM.extab : ALIGN(4) {
        *(.ARM.extab*)
    } > kflash

    .ARM.exidx : ALIGN(4) {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
    } > kflash

    .tdata : {
        _stdata = ABSOLUTE(.);
        *(.tdata .tdata.* .gnu.linkonce.td.*);
        _etdata = ABSOLUTE(.);
    } > kflash

    .tbss : {
        _stbss = ABSOLUTE(.);
        *(.tbss .tbss.* .gnu.linkonce.tb.* .tcommon);
        _etbss = ABSOLUTE(.);
    } > kflash

    _eronly = ABSOLUTE(.);

    .data : ALIGN(4) {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > ksram AT > kflash

    .bss : ALIGN(4) {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > ksram

    /* Stabs debugging sections. */

    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}
