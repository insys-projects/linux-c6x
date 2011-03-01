/*
 *  linux/arch/c6x/platforms/include/mach/pll-c6474.h
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

#ifndef __MACH_C6X_PLL_C6474_H
#define __MACH_C6X_PLL_C6474_H

#define ARCH_PLL1_BASE          0x029A0000

/*
 * Datasheet recommends a wait for 4 CLKIN cycles to ensure that the
 * PLL has switched to bypass mode. Delay of 1us ensures we are good for
 * all > 4MHz CLKIN inputs. Typically the input is ~25MHz.
 * Units are micro seconds.
 */
#define PLL_BYPASS_TIME		1

#define PLL_RESET_TIME		256
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
		.flags = CLK_PLL,				\
	};							\
								\
	static struct clk pll1_sysclk7 = {			\
		.name = "pll1_sysclk7",				\
		.parent = &pll1_clk,				\
		.flags = CLK_PLL | FIXED_DIV_PLL,		\
		.div = 1,					\
	};							\
								\
	static struct clk pll1_sysclk9 = {			\
		.name = "pll1_sysclk9",				\
		.parent = &pll1_clk,				\
		.flags = CLK_PLL | FIXED_DIV_PLL,		\
		.div = 3,					\
	};							\
								\
	static struct clk pll1_sysclk10 = {			\
		.name = "pll1_sysclk10",			\
		.parent = &pll1_clk,				\
		.flags = CLK_PLL | FIXED_DIV_PLL,		\
		.div = 6,					\
	};							\
								\
	static struct clk pll1_sysclk11 = {			\
		.name = "pll1_sysclk11",			\
		.parent = &pll1_clk,				\
		.flags = CLK_PLL,				\
		.div = PLLDIV11,				\
	};							\
								\
	static struct clk pll1_sysclk12 = {			\
		.name = "pll1_sysclk12",			\
		.parent = &pll1_clk,				\
		.flags = CLK_PLL | FIXED_DIV_PLL,		\
		.div = 2,					\
	};							\
								\
	static struct clk pll1_sysclk13 = {			\
		.name = "pll1_sysclk13",			\
		.parent = &pll1_clk,				\
		.flags = CLK_PLL,				\
		.div = PLLDIV13,				\
	};							\
								\
	static struct clk i2c_clk = {				\
		.name = "i2c",					\
		.parent = &pll1_sysclk10,			\
	};							\
								\
	static struct clk mcbsp1_clk = {			\
		.name = "mcbsp1",				\
		.parent = &pll1_sysclk10,			\
	};							\
								\
	static struct clk mcbsp2_clk = {			\
		.name = "mcbsp2",				\
		.parent = &pll1_sysclk10,			\
	};							\
								\
	static struct clk core_clk = {				\
		.name = "core",					\
		.parent = &pll1_sysclk7,			\
	};							\
								\
	static struct clk watchdog_clk = {			\
		.name = "watchdog",				\
		.parent = &pll1_sysclk10,			\
	}

#define SOC_CLK()					\
	CLK(NULL, "pll1", &pll1_clk),			\
	CLK(NULL, "pll1_sysclk7", &pll1_sysclk7),	\
	CLK(NULL, "pll1_sysclk9", &pll1_sysclk9),	\
	CLK(NULL, "pll1_sysclk10", &pll1_sysclk10),	\
	CLK(NULL, "pll1_sysclk11", &pll1_sysclk11),	\
	CLK(NULL, "pll1_sysclk12", &pll1_sysclk12),	\
	CLK(NULL, "pll1_sysclk13", &pll1_sysclk13),	\
	CLK(NULL, "core", &core_clk),			\
	CLK("i2c_davinci.1", NULL, &i2c_clk),		\
	CLK("mcbsp.1", NULL, &mcbsp1_clk),		\
	CLK("mcbsp.2", NULL, &mcbsp2_clk),		\
	CLK("watchdog", NULL, &watchdog_clk)

#endif /* __MACH_PLL_C6474_H */
