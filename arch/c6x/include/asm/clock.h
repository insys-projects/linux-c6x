/*
 * TI C64X clock definitions
 *
 * Copyright (C) 2010 Texas Instruments.
 * Contributed by: Mark Salter <msalter@redhat.com>
 *
 * Copied heavily from arm/mach-davinci/clock.h, so:
 *
 * Copyright (C) 2006-2007 Texas Instruments.
 * Copyright (C) 2008-2009 Deep Root Systems, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_C6X_CLOCK_H
#define __ARCH_C6X_CLOCK_H

#include <mach/pll.h>

#ifndef __ASSEMBLER__

#include <linux/list.h>
#include <asm/clkdev.h>

struct pll_data {
	u32 phys_base;
	void __iomem *base;
	u32 num;
	u32 flags;
	u32 input_rate;
};
#define PLL_HAS_PREDIV          0x01

struct clk {
	struct list_head	node;
	struct module		*owner;
	const char		*name;
	unsigned long		rate;
	int			usecount;
	u32			flags;
	struct clk              *parent;
	struct list_head	children; 	/* list of children */
	struct list_head	childnode;	/* parent's child list node */
	struct pll_data         *pll_data;
	u32                     div;
	unsigned long (*recalc) (struct clk *);
	int (*set_rate) (struct clk *clk, unsigned long rate);
	int (*round_rate) (struct clk *clk, unsigned long rate);
};

/* Clock flags: SoC-specific flags start at BIT(16) */
#define ALWAYS_ENABLED		BIT(1)
#define CLK_PLL			BIT(2) /* PLL-derived clock */
#define PRE_PLL                 BIT(3) /* source is before PLL mult/div */
#define FIXED_DIV_PLL           BIT(4) /* fixed divisor from PLL */
#define FIXED_RATE_PLL          BIT(5) /* fixed ouput rate PLL */

#define CLK(dev, con, ck) 	\
	{			\
		.dev_id = dev,	\
		.con_id = con,	\
		.clk = ck,	\
	}			\

int c6x_clk_init(struct clk_lookup *clocks);
int c6x_set_pllrate(struct pll_data *pll, unsigned int prediv,
		    unsigned int mult);

#endif

#endif
