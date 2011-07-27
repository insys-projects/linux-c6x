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

#include <asm/io.h>
#include <mach/dscr.h>

#define dscr_set_reg(reg, val) __raw_writel((u32) val, reg)
#define dscr_get_reg(reg)      __raw_readl(reg)

#if defined(CONFIG_SOC_TMS320C6455) || defined(CONFIG_SOC_TMS320C6474) || defined(CONFIG_SOC_TMS320C6472)
static inline void dscr_set_device(unsigned int w, unsigned int reg)
{
	while (dscr_get_reg(reg) != w) {
		dscr_set_reg(DSCR_PERLOCK, DSCR_LOCKVAL);
		dscr_set_reg(reg, w);
	}
}
#endif
#endif /*__ASM_C6X_DSCR_H */
