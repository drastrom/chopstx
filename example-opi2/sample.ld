/*
 * Orange Pi PC memory setup.
 */
__main_stack_size__      = 0x0100; /* Idle thread */
__process0_stack_size__  = 0x0100; /* Main program           */
__process1_stack_size__  = 0x0100; /* first thread progprog */
__process2_stack_size__  = 0x0100; /* second thread progprog */
__process3_stack_size__  = 0x0100; /* third thread progprog */
__svc_stack_size__	 = 0x0400;  /* SVC exception stack size */
__exc_stack_size__	 = 0x0100;  /* Other exception stack size */

MEMORY
{
    prog : org = 0x00004000, len = 48k
}

__prog_start__           = ORIGIN(prog);
__prog_size__            = 48k;
__prog_end__             = __prog_start__ + __prog_size__;
__svc_stack_end__	= __prog_start__;
__irq_stack_end__	= __svc_stack_end__ - 4 * __svc_stack_size__;
__fiq_stack_end__	= __irq_stack_end__ - 4 * __exc_stack_size__;
__abt_stack_end__	= __fiq_stack_end__ - 4 * __exc_stack_size__;
__und_stack_end__	= __abt_stack_end__ - 4 * __exc_stack_size__;

EXTERN (entry)

SECTIONS
{
    . = __prog_start__;

    _text = .;

    .text : ALIGN(16) SUBALIGN(16)
    {
        KEEP(*(.text.startup.0))
        *(.text.startup.*)
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
	. = ALIGN(8);
    } > prog

    .ARM.extab : {*(.ARM.extab* .gnu.linkonce.armextab.*)} > prog

    .ARM.exidx : {
        PROVIDE(__exidx_start = .);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        PROVIDE(__exidx_end = .);
     } > prog

    .eh_fproge_hdr : {*(.eh_fproge_hdr)} > prog

    .eh_fproge : ONLY_IF_RO {*(.eh_fproge)} > prog

    .textalign : ONLY_IF_RO { . = ALIGN(8); } > prog

    _etext = .;

    .data :
    {
        . = ALIGN(4);
        PROVIDE(_data = .);
        *(.data)
        . = ALIGN(4);
        *(.data.*)
        . = ALIGN(4);
        *(.progtext)
        . = ALIGN(4);
        *(.note.gnu.build-id)
        . = ALIGN(4);
        PROVIDE(_edata = .);
    } > prog

    .bss :
    {
        . = ALIGN(4);
        PROVIDE(_bss_start = .);
        *(.bss)
        . = ALIGN(4);
        *(.bss.*)
        . = ALIGN(4);
        *(COMMON)
        . = ALIGN(4);
        PROVIDE(_bss_end = .);
        . = ALIGN(16384);
        PROVIDE(_translation_table = .);
    } > prog

    .process_stack :
    {
        . = ALIGN(8);
        __process3_stack_base__ = .;
        . += __process3_stack_size__;
        . = ALIGN(8);
        __process3_stack_end__ = .;
        __process2_stack_base__ = .;
        . += __process2_stack_size__;
        . = ALIGN(8);
        __process2_stack_end__ = .;
        __process1_stack_base__ = .;
        . += __process1_stack_size__;
        . = ALIGN(8);
        __process1_stack_end__ = .;
        __process0_stack_base__ = .;
        . += __process0_stack_size__;
        . = ALIGN(8);
        __process0_stack_end__ = .;
    } > prog

    .main_stack :
    {
        . = ALIGN(8);
        __main_stack_base__ = .;
        . += __main_stack_size__;
        . = ALIGN(8);
        __main_stack_end__ = .;
    } > prog

    PROVIDE(end = .);
    _end            = .;

    /* DWARF debug sections.
       Symbols in the DWARF debugging sections are relative to the beginning
       of the section so we begin them at 0.  */
    /* DWARF 1 */
    .debug          0 : { *(.debug) }
    .line           0 : { *(.line) }
    /* GNU DWARF 1 extensions */
    .debug_srcinfo  0 : { *(.debug_srcinfo) }
    .debug_sfnames  0 : { *(.debug_sfnames) }
    /* DWARF 1.1 and DWARF 2 */
    .debug_aranges  0 : { *(.debug_aranges) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    /* DWARF 2 */
    .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
    .debug_abbrev   0 : { *(.debug_abbrev) }
    .debug_line     0 : { *(.debug_line .debug_line.* .debug_line_end ) }
    .debug_fproge    0 : { *(.debug_fproge) }
    .debug_str      0 : { *(.debug_str) }
    .debug_loc      0 : { *(.debug_loc) }
    .debug_macinfo  0 : { *(.debug_macinfo) }
    /* SGI/MIPS DWARF 2 extensions */
    .debug_weaknames 0 : { *(.debug_weaknames) }
    .debug_funcnames 0 : { *(.debug_funcnames) }
    .debug_typenames 0 : { *(.debug_typenames) }
    .debug_varnames  0 : { *(.debug_varnames) }
    /* DWARF 3 */
    .debug_pubtypes 0 : { *(.debug_pubtypes) }
    .debug_ranges   0 : { *(.debug_ranges) }
    /* DWARF Extension.  */
    .debug_macro    0 : { *(.debug_macro) }
    .gnu.attributes 0 : { KEEP (*(.gnu.attributes)) }
    /DISCARD/ : { *(.note.GNU-stack) *(.gnu_debuglink) *(.gnu.lto_*) *(.gnu_object_only) }
}

__heap_base__   = _end;
__heap_end__    = __prog_end__;
