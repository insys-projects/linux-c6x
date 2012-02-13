#include <asm-generic/vmlinux.lds.h>
OUTPUT_ARCH(tic6x)
ENTRY(startup)
SECTIONS
{
	. = 0;
	.text : {
		_text = .;
		*(.text)
		*(.text.*)
		_etext = .;
	}
	.rodata.compressed : {
		_sdata_compressed = . ;
		*(.rodata.compressed)
		_edata_compressed = . ;
	}
	.data : {
		_sdata  =  .;
		. = ALIGN(32);
		*(.rodata)
		*(.rodata.*)
	        *(.const)
		*(.switch)
		*(.data)
		*(.data.*)
		. = ALIGN(32); 
		_edata  =  .;
	}
	. = ALIGN(256);
	.bss : {
		_bss_start = .;
		*(.bss)
		*(.sbss)
		*(.far)
		. = ALIGN(32);
		_bss_end = .;
		. = ALIGN(4096);
		. += 4096;
		_end = . ;
	}
}
