/*
 *  linux/include/asm-c6x/pm.h
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
#ifndef __ASM_C6X_PM_H
#define __ASM_C6X_PM_H

#define PWRD_NONE        0x00
#define PWRD_PD1A        0x09
#define PWRD_PD1B        0x11
#define PWRD_PD2         0x1a
#define PWRD_PD3         0x1c
#define PWRD_IDLE        0xff

extern cregister volatile unsigned int CSR; /* Control Status Register */

#define pwrd_set(pwrd)   (CSR |= ((pwrd) & 0xff) << 10)
#define do_idle()        asm(" IDLE")

#define PWR_PDCTL_BASE   0x019c0200

#define PWR_PDCTL_MCBSP2 0x10
#define PWR_PDCTL_MCBSP1 0x08
#define PWR_PDCTL_MCBSP0 0x04
#define PWR_PDCTL_EMIF   0x02
#define PWR_PDCTL_DMA    0x01
#define PWR_PDCTL_ALL    0x1f

#define pwr_pdctl_setbit(val) \
        *((volatile unsigned int *) PWR_PDCTL_BASE) |= (unsigned int) (val)
	    
#define pwr_pdctl_clearbit(val) \
        *((volatile unsigned int *) PWR_PDCTL_BASE) &= ~((unsigned int) (val))
        
#define pwr_pdctl_set(val) \
        *((volatile unsigned int *) PWR_PDCTL_BASE) = (unsigned int) (val)
        
#define pwr_pdctl_get() \
        *((volatile unsigned int *) PWR_PDCTL_BASE)

#endif /* __ASM_C6X_PM_H */

