/*
 *  linux/include/asm-c6x/flat.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_FLAT_H
#define __ASM_C6X_FLAT_H

struct flat_hdr {
	char magic[4];
	unsigned long rev;
	unsigned long entry; /* Offset of first executable instruction with text segment from beginning of file*/
	unsigned long data_start; /* Offset of data segment from beginning of file*/
	
	unsigned long data_end; /* Offset of end of data segment from beginning of file*/
	unsigned long bss_end; /* Offset of end of bss segment from beginning of file*/
				/* (It is assumed that data_end through bss_end forms the
				    bss segment.) */
	unsigned long stack_size; /* Size of stack, in bytes */
	unsigned long reloc_start; /* Offset of relocation records from beginning of file */
	
	unsigned long reloc_count; /* Number of relocation records */
	
	unsigned long filler[7]; /* Reservered, set to zero */
};

#define FLAT_RELOC_TYPE_TEXT 0
#define FLAT_RELOC_TYPE_DATA 1
#define FLAT_RELOC_TYPE_BSS 2

struct flat_reloc {
	unsigned long type : 2; 
	unsigned long offset : 30;
};

#endif /* __ASM_C6X_FLAT_H */
