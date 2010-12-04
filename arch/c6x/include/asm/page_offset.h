/*
 *  linux/include/asm-c6x/page_offset.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */
#ifndef _ASM_C6X_PAGE_OFFSET_H
#define _ASM_C6X_PAGE_OFFSET_H
#include <mach/hardware.h>

/* This handles the memory map */
#ifdef CONFIG_PAGE_OFFSET
#define PAGE_OFFSET_RAW		CONFIG_PAGE_OFFSET
#else
#define PAGE_OFFSET_RAW		RAM_DDR2_CE0
#endif

/* Maximum size for the kernel code */
#define KERNEL_TEXT_LEN         0x07ffffff

#endif /* _ASM_C6X_PAGE_OFFSET_H */
