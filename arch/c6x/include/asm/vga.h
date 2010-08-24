/*
 *  linux/include/asm-c6x/vga.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2006, 2009 Texas Instruments Incorporated
 *  Author: Nicolas Videau (nicolas.videau@virtuallogix.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef ASM_C6X_VGA_H
#define ASM_C6X_VGA_H

#define VGA_MAP_MEM(x)	(x)

#define vga_readb(x)	(*((volatile unsigned char *)x))
#define vga_writeb(x,y)	(*((volatile unsigned char *)y) = (x))

#endif /* ASM_C6X_VGA_H */
