/*
 *  linux/arch/c6x/platforms/include/mach/pll-c6455.h
 *
 *  PLL definitions for Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010, 2011 Texas Instruments Incorporated
 *  Author: Mark Salter <msalter@redhat.com>
 *          Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef __MACH_C6X_PLL_C6455_H
#define __MACH_C6X_PLL_C6455_H

#define ARCH_PLL1_BASE          0x029A0000
#define ARCH_PLL2_BASE          0x029C0000

/*
 * Datasheet recommends a wait for 4 CLKIN cycles to ensure that the
 * PLL has switched to bypass mode. Delay of 1us ensures we are good for
 * all > 4MHz CLKIN inputs. Typically the input is ~25MHz.
 * Units are micro seconds.
 */
#define PLL_BYPASS_TIME		1

#define PLL_RESET_TIME		128
#define PLL_LOCK_TIME		2000

/* Get multiplier on C64x+ architecture */
#define get_main_pll_mult(base) (((__raw_readl((base) + PLLM)) & PLLM_PLLM_MASK) + 1)

/*
 * Standard PLL and clock definitions for this SoC
 */
#define SOC_CLK_DEF(clkin)					\
	static struct pll_data pll1_data = {			\
		.num       = 1,					\
		.phys_base = ARCH_PLL1_BASE,			\
	};							\
								\
	static struct clk clkin1 = {				\
		.name = "clkin1",				\
		.rate = (clkin),				\
		.node = LIST_HEAD_INIT(clkin1.node),		\
		.children = LIST_HEAD_INIT(clkin1.children),	\
		.childnode = LIST_HEAD_INIT(clkin1.childnode),	\
	};							\
								\
	static struct clk pll1_clk = {				\
		.name = "pll1",					\
		.parent = &clkin1,				\
		.pll_data = &pll1_data,				\
		.flags = CLK_PLL | PLL_HAS_PREDIV,		\
	};							\
								\
	static struct clk pll1_sysclk2 = {			\
		.name = "pll1_sysclk2",				\
		.parent = &pll1_clk,				\
		.flags = CLK_PLL | FIXED_DIV_PLL,		\
		.div = 3,					\
	};							\
								\
	static struct clk pll1_sysclk3 = {			\
		.name = "pll1_sysclk3",				\
		.parent = &pll1_clk,				\
		.flags = CLK_PLL | FIXED_DIV_PLL,		\
		.div = 6,					\
	};							\
								\
	static struct clk pll1_sysclk4 = {			\
		.name = "pll1_sysclk4",				\
		.parent = &pll1_clk,				\
		.flags = CLK_PLL,				\
		.div = PLLDIV4,					\
	};							\
								\
	static struct clk pll1_sysclk5 = {			\
		.name = "pll1_sysclk5",				\
		.parent = &pll1_clk,				\
		.flags = CLK_PLL,				\
		.div = PLLDIV5,					\
	};							\
								\
	static struct clk i2c_clk = {				\
		.name = "i2c",					\
		.parent = &pll1_sysclk3,			\
	};							\
								\
	static struct clk watchdog_clk = {			\
		.name = "watchdog",				\
		.parent = &pll1_sysclk3,			\
	};							\
								\
	static struct clk core_clk = {				\
		.name = "core",					\
		.parent = &pll1_clk,				\
	}

#define SOC_CLK()					\
	CLK(NULL, "pll1", &pll1_clk),			\
	CLK(NULL, "pll1_sysclk2", &pll1_sysclk2),	\
	CLK(NULL, "pll1_sysclk3", &pll1_sysclk3),	\
	CLK(NULL, "pll1_sysclk4", &pll1_sysclk4),       \
	CLK(NULL, "pll1_sysclk5", &pll1_sysclk5),       \
	CLK(NULL, "core", &core_clk),                   \
	CLK("i2c_davinci.1", NULL, &i2c_clk),           \
	CLK("watchdog", NULL, &watchdog_clk)

#endif /* __MACH_PLL_C6455_H */
