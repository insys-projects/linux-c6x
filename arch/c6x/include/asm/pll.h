/*
 *  linux/include/asm-c6x/pll.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2006, 2009, 2010 Texas Instruments Incorporated
 *  Author: Nicolas Videau (nicolas.videau@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_PLL_H
#define __ASM_C6X_PLL_H

#include <asm/io.h>
#include <mach/pll.h>

#ifdef ARCH_PLL1_BASE
#define pll1_set_reg(reg, val) \
	__raw_writel((val), (const volatile void *)(ARCH_PLL1_BASE + (reg)))
        
#define pll1_get_reg(reg) \
	__raw_readl((const volatile void *)(ARCH_PLL1_BASE + (reg)))

#define pll1_clearbit_reg(reg, val) \
	pll1_set_reg((reg), pll1_get_reg(reg) & ~((u32)(val)))

#define pll1_setbit_reg(reg, val) \
	pll1_set_reg((reg), pll1_get_reg(reg) | (u32)(val))

static inline void pll1_wait_gostat(void)
{
	while(pll1_get_reg(PLLSTAT) & PLLSTAT_GOSTAT);
}
#endif /* ARCH_PLL1_BASE */

#ifdef ARCH_PLL2_BASE
#define pll2_set_reg(reg, val) \
	__raw_writel((val), (const volatile void *)(ARCH_PLL2_BASE + (reg)))
        
#define pll2_get_reg(reg) \
	__raw_readl((const volatile void *)(ARCH_PLL2_BASE + (reg)))

#define pll2_clearbit_reg(reg, val) \
	pll2_set_reg((reg), pll2_get_reg(reg) & ~((u32)(val)))

#define pll2_setbit_reg(reg, val) \
	pll2_set_reg((reg), pll2_get_reg(reg) | (u32)(val))

static inline void pll2_wait_gostat(void)
{
	while(pll2_get_reg(PLLSTAT) & PLLSTAT_GOSTAT);
}
#endif /* ARCH_PLL2_BASE */

#ifdef ARCH_PLL3_BASE
#define pll3_set_reg(reg, val) \
	__raw_writel((val), (const volatile void *)(ARCH_PLL3_BASE + (reg)))
        
#define pll3_get_reg(reg) \
	__raw_readl((const volatile void *)(ARCH_PLL3_BASE + (reg)))

#define pll3_clearbit_reg(reg, val) \
	pll3_set_reg((reg), pll3_get_reg(reg) & ~((u32)(val)))

#define pll3_setbit_reg(reg, val) \
	pll3_set_reg((reg), pll3_get_reg(reg) | (u32)(val))

static inline void pll3_wait_gostat(void)
{
	while(pll3_get_reg(PLLSTAT) & PLLSTAT_GOSTAT);
}
#endif /* ARCH_PLL2_BASE */

#endif /*__ASM_C6X_PLL_H */
