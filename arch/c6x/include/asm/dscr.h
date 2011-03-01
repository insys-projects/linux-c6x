/*
 *  linux/include/asm-c6x/dscr.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2006, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_DSCR_H
#define __ASM_C6X_DSCR_H

#include <mach/dscr.h>

#define dscr_set_reg(reg, val) \
        *((volatile u32 *) (reg)) = (u32) (val)
        
#define dscr_get_reg(reg) \
        *((volatile u32 *) (reg))

#if defined(CONFIG_SOC_TMS320C6455) || defined(CONFIG_SOC_TMS320C6474) || defined(CONFIG_SOC_TMS320C6472)
static inline void dscr_set_device(unsigned int w, unsigned int reg)
{
	while (dscr_get_reg(reg) != w) {
		dscr_set_reg(DSCR_PERLOCK, DSCR_LOCKVAL);
		dscr_set_reg(reg, w);
	}
}
#endif

#if defined(CONFIG_SOC_TMS320C6670) || defined(CONFIG_SOC_TMS320C6678)
static inline void dscr_set_device(unsigned int w, unsigned int reg)
{
	/* Unlock DSCR boot config */
	dscr_set_reg(DSCR_KICK0, DSCR_KICK0_KEY);
	dscr_set_reg(DSCR_KICK1, DSCR_KICK1_KEY);

	dscr_set_reg(reg, w);

	/* Lock DSCR */
	dscr_set_reg(DSCR_KICK0, 0);
	dscr_set_reg(DSCR_KICK1, 0);
}
#endif

#endif /*__ASM_C6X_DSCR_H */
