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

#elif defined(CONFIG_SOC_TMS320C6457)
#define ARCH_PLL1_BASE          0x029A0000

/*
 * Datasheet recommends a wait for 4 CLKIN cycles to ensure that the
 * PLL has switched to bypass mode. Delay of 1us ensures we are good for
 * all > 4MHz CLKIN inputs. Typically the input is ~25MHz.
 * Units are micro seconds.
 */
#define PLL_BYPASS_TIME		1

#define PLL_RESET_TIME		1000
#define PLL_LOCK_TIME		2000

#elif defined(CONFIG_SOC_TMS320C6472)
#define ARCH_PLL1_BASE          0x029A0000
#define ARCH_PLL2_BASE          0x029C0000
#define ARCH_PLL3_BASE          0x029C0400

/*
 * Datasheet recommends a wait for 4 CLKIN cycles to ensure that the
 * PLL has switched to bypass mode. Delay of 1us ensures we are good for
 * all > 4MHz CLKIN inputs. Typically the input is ~25MHz.
 * Units are micro seconds.
 */
#define PLL_BYPASS_TIME		1

#define PLL_RESET_TIME		256
#define PLL_LOCK_TIME		2000

#elif defined(CONFIG_SOC_TMS320C6474)
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

#elif defined(CONFIG_SOC_TMS320C6670)
#define ARCH_PLL1_BASE          0x02310000

#elif defined(CONFIG_SOC_TMS320C6678)
#define ARCH_PLL1_BASE          0x02310000

/*
 * Datasheet recommends a wait for 4 CLKIN cycles to ensure that the
 * PLL has switched to bypass mode. Delay of 1us ensures we are good for
 * all > 4MHz CLKIN inputs. Typically the input is ~25MHz.
 * Units are micro seconds.
 */
#define PLL_BYPASS_TIME		1

#define PLL_RESET_TIME		256
#define PLL_LOCK_TIME		2000

#endif

/* PLL/Reset register offsets */
#define PLLCTL          0x100
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
