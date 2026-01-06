OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
/* ENTRY(_start) */

MEMORY
{
	sram : org = 0x00020000, len = 32K
	ram  : org = 0x40000000, len = 16M
	dma  : org = 0x41000000, len = 8M
	heap : org = 0x41800000, len = 104M
}

SECTIONS
{
    .text :
    {
    	. = ALIGN(4);
    	PROVIDE(__image_start = .);
	    __text_start = .;
        *(.vectors)
        *(.text)
        *(.text.*)

        /* section information for utest */
        . = ALIGN(4);
        __rt_utest_tc_tab_start = .;
        KEEP(*(UtestTcTab))
        __rt_utest_tc_tab_end = .;
        
        /* section information for finsh shell */
        . = ALIGN(4);
        __fsymtab_start = .;
        KEEP(*(FSymTab))
        __fsymtab_end = .;
        
        . = ALIGN(4);
        __vsymtab_start = .;
        KEEP(*(VSymTab))
        __vsymtab_end = .;

        /* section information for var export */
        . = ALIGN(4);
        __ve_table_start = .;
        KEEP(*(SORT(*.VarExpTab.*)))
        __ve_table_end = .;

        /* section information for modules */
        . = ALIGN(4);
        __rtmsymtab_start = .;
        KEEP(*(RTMSymTab))
        __rtmsymtab_end = .;

        /* section information for initialization */
        . = ALIGN(4);
        __rt_init_start = .;
        KEEP(*(SORT(.rti_fn*)))
        __rt_init_end = .;
        
        /* section information for usbh class */
		. = ALIGN(4);
		__usbh_class_info_start__ = .;
		KEEP(*(.usbh_class_info))
		__usbh_class_info_end__ = .;
    } >ram
    __text_end = .;

    __exidx_start = .;
    .ARM.exidx : 
    { 
    	*(.ARM.exidx* .gnu.linkonce.armexidx.*) 
	} >ram
    __exidx_end = .;

    __rodata_start = .;
    .rodata : 
    { 
    	*(.rodata) 
    	*(.rodata.*) 
    } >ram
    __rodata_end = .;

    . = ALIGN(4);
    .ctors :
    {
        PROVIDE(__ctors_start__ = .);
        KEEP(*(SORT(.ctors.*)))
        KEEP(*(.ctors))
        PROVIDE(__ctors_end__ = .);
    } >ram

    .dtors :
    {
        PROVIDE(__dtors_start__ = .);
        KEEP(*(SORT(.dtors.*)))
        KEEP(*(.dtors))
        PROVIDE(__dtors_end__ = .);
    } >ram

    . = ALIGN(8);
    __data_start = .;
    .data :
    {
        *(.data)
        *(.data.*)
    } >ram
    __data_end = .;
    PROVIDE(__image_end = .);
    PROVIDE(__image_size = __image_end - __image_start);
    
    .dma (NOLOAD) :
    {
    	. = ALIGN(32);
    	*(.noncacheable)
    } >dma

    .bss (NOLOAD) :
    {
        . = ALIGN(8);
        __bss_start = .;
        
        *(.bss)
        *(.bss.*)
        *(COMMON)
    } >heap
    . = ALIGN(32);
    __bss_end = .;

    /* Stabs debugging sections.  */
    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }

    _end = .;
}
