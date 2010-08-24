/*
 *  linux/include/asm-c6x/wugen.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2007, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@virtuallogix.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_WUGEN_H
#define __ASM_C6X_WUGEN_H

#include <asm/hardware.h>

#define WUGEN_REVISION    0x000
#define WUGEN_SYSCONFIG   0x008
#define WUGEN_MEVT0       0x060
#define WUGEN_MEVT1       0x064
#define WUGEN_MEVT2       0x068
#define WUGEN_MEVT3       0x06c
#define WUGEN_MEVTCLR0    0x070
#define WUGEN_MEVTCLR1    0x074
#define WUGEN_MEVTCLR2    0x078
#define WUGEN_MEVTCLR3    0x07c
#define WUGEN_MEVTSET0    0x080
#define WUGEN_MEVTSET1    0x084
#define WUGEN_MEVTSET2    0x088
#define WUGEN_MEVTSET3    0x08c
#define WUGEN_PENDEVT0    0x090
#define WUGEN_PENDEVT1    0x094
#define WUGEN_PENDEVT2    0x098
#define WUGEN_PENDEVT3    0x09c
#define WUGEN_PENDEVTCLR0 0x100
#define WUGEN_PENDEVTCLR1 0x104
#define WUGEN_PENDEVTCLR2 0x108
#define WUGEN_PENDEVTCLR3 0x10c

#define wugen_setbit_reg(reg, val) \
        *((volatile unsigned int *) (WUGEN_REG_BASE + (reg))) |= (unsigned int) (val)
	    
#define wugen_clearbit_reg(reg, val) \
        *((volatile unsigned int *) (WUGEN_REG_BASE + (reg))) &= ~((unsigned int) (val))
        
#define wugen_set_reg(reg, val) \
        *((volatile unsigned int *) (WUGEN_REG_BASE + (reg))) = (unsigned int) (val)
        
#define wugen_get_reg(reg) \
        *((volatile unsigned int *) (WUGEN_REG_BASE + (reg)))

#endif /* __ASM_C6X_WUGEN_H */
