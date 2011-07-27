/*
 *  linux/arch/c6x/platforms/include/mach/pll.h
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

#ifndef __MACH_C6X_PLL_H
#define __MACH_C6X_PLL_H

#if defined(CONFIG_SOC_TMS320C6455)
#include <mach/pll-c6455.h>
#elif defined(CONFIG_SOC_TMS320C6457)
#include <mach/pll-c6457.h>
#elif defined(CONFIG_SOC_TMS320C6472)
#include <mach/pll-c6472.h>
#elif defined(CONFIG_SOC_TMS320C6474)
#include <mach/pll-c6474.h>
#elif defined(CONFIG_SOC_TMS320C6670)
#include <mach/pll-c6670.h>
#elif defined(CONFIG_SOC_TMS320C6678)
#include <mach/pll-c6678.h>
#else
#error "No machine PLL definitions"
#endif

/* PLL/Reset register offsets */
#define PLLCTL          0x100
#define SECCTL          0x108
#define PLLM		0x110
#define PREDIV          0x114
#define PLLDIV1         0x118
#define PLLDIV2         0x11c
#define PLLDIV3         0x120
#define PLLPOSTDIV      0x128
#define PLLCMD		0x138
#define PLLSTAT		0x13c
#define PLLALNCTL	0x140
#define PLLDCHANGE	0x144
#define PLLCKEN		0x148
#define PLLCKSTAT	0x14c
#define PLLSYSTAT	0x150
#define PLLDIV4         0x160
#define PLLDIV5         0x164
#define PLLDIV6         0x168
#define PLLDIV7         0x16c
#define PLLDIV8         0x170
#define PLLDIV9         0x174
#define PLLDIV10        0x178
#define PLLDIV11        0x17c
#define PLLDIV12        0x180
#define PLLDIV13        0x184
#define PLLDIV14        0x188
#define PLLDIV15        0x18c
#define PLLDIV16        0x190

/* PLLM register bits */
#define PLLM_PLLM_MASK  0xff
#define PLLM_VAL(x)     ((x) - 1)

/* PREDIV register bits */
#define PLLPREDIV_EN	BIT(15)
#define PLLPREDIV_VAL(x) ((x) - 1)

/* PLLCTL register bits */
#define PLLCTL_PLLEN    BIT(0)
#define PLLCTL_PLLPWRDN	BIT(1)
#define PLLCTL_PLLRST	BIT(3)
#define PLLCTL_PLLDIS	BIT(4)
#define PLLCTL_PLLENSRC	BIT(5)
#define PLLCTL_CLKMODE  BIT(8)

/* PLLCMD register bits */
#define PLLCMD_GOSTAT	BIT(0)

/* PLLSTAT register bits */
#define PLLSTAT_GOSTAT	BIT(0)

/* PLLDIV register bits */
#define PLLDIV_EN       BIT(15)
#define PLLDIV_RATIO_MASK 0x1f
#define PLLDIV_RATIO(x) ((x) - 1)

#endif /* __MACH_C6X_PLL_H */
