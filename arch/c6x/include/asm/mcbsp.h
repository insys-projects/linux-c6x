/*
 *  linux/include/asm-c6x/mcbsp.h
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
#ifndef __ASM_ARCH_MCBSP_H
#define __ASM_ARCH_MCBSP_H

#include <asm/hardware.h>

#if defined(CONFIG_TMS320DM644X) || defined(CONFIG_TMS320DM643X)
/* DaVinci DSPs have their own methods*/
#include <asm/davinci_mcbsp.h>
#else
#define DRR     0x00
#define DXR     0x04
#define SPCR    0x08
#define RCR     0x0c
#define XCR     0x10
#define SRGR    0x14
#define MCR     0x18
#define RCERE0  0x1c
#define XCERE0  0x20
#define PCR     0x24
#define RCERE1  0x28
#define XCERE1  0x2c
#define RCERE2  0x30
#define XCERE2  0x34
#define RCERE3  0x38
#define XCERE3  0x3c

#ifdef  MCBSP1_BASE_ADDR 
#define MCBSP_REG_BASE(num) ((unsigned int) (MCBSP0_BASE_ADDR + \
					     ((MCBSP1_BASE_ADDR - MCBSP0_BASE_ADDR) * num)))
#else
#define MCBSP_REG_BASE(num) MCBSP0_BASE_ADDR
#endif

#define mcbsp_setbit_reg(reg, num, val) \
        *((volatile unsigned int *) (MCBSP_REG_BASE(num) + (reg))) |= (unsigned int) (val)
	    
#define mcbsp_clearbit_reg(reg, num, val) \
        *((volatile unsigned int *) (MCBSP_REG_BASE(num) + (reg))) &= ~((unsigned int) (val))
        
#define mcbsp_set_reg(reg, num, val) \
        *((volatile unsigned int *) (MCBSP_REG_BASE(num) + (reg))) = (unsigned int) (val)
        
#define mcbsp_get_reg(reg, num) \
        *((volatile unsigned int *) (MCBSP_REG_BASE(num) + (reg)))

#endif /* CONFIG_TMS320DM644X || CONFIG_TMS320DM643X */
#endif /* __ASM_ARCH_MCBSP_H */ 
