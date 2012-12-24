/*
 *  linux/include/asm-c6x/irq.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2006, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_IRQ_H_
#define __ASM_C6X_IRQ_H_

#include <linux/types.h>
#include <asm/hardware.h>
#include <asm/percpu.h>

#include <mach/irq.h>
#include <mach/pci.h>

#define irq_canonicalize(irq)  (irq)

#ifdef CONFIG_INTC_C64XPLUS
#define NR_IRQS (NR_SOC_IRQS + MSI_NR_IRQS + NR_BOARD_IRQS)
#else
#define NR_IRQS (NR_SYS_IRQS + NR_BOARD_IRQS)
#endif

/*
 * Number of C6x interrupt vectors.
 *
 * There are 16 vectors. One each is used by Reset and NMI. Two are reserved.
 * The remaining 12 vectors are used to route SoC interrupt sources. These
 * interrupt vectors are prioritized with INT4 having the highest priority
 * and INT15 having the lowest.
 *
 * The C64x+ megamodule provides a way to combine SoC IRQ sources into a single
 * IRQ vector. There are four combined sources, each of which feed into one of
 * the 12 general interrupt vectors. The remaining 8 vectors can each route a
 * single SoC interrupt directly.
 *
 */
#define NR_SYS_IRQS 16

/*
 * Processor interrupt vector definitions
 * Interrupt sources are prioritized from INT0 (highest) to INT15 (lowest).
 */
#define INT0                0    /* RESET */
#define INT1		    1	 /* NMI */
#define INT2		    2	 /* Reserved */
#define INT3		    3	 /* Reserved */
#define INT4		    4	 /* level 4 interrupt */
#define INT5		    5	 /* level 5 interrupt */
#define INT6		    6	 /* level 6 interrupt */
#define INT7		    7	 /* level 7 interrupt */
#define INT8		    8	 /* level 8 interrupt */
#define INT9		    9	 /* level 9 interrupt */
#define INT10		    10   /* level 10 interrupt */
#define INT11		    11   /* level 11 interrupt */
#define INT12		    12   /* level 12 interrupt */
#define INT13		    13   /* level 13 interrupt */
#define INT14		    14   /* level 14 interrupt */
#define INT15		    15   /* level 15 interrupt */

#ifdef CONFIG_INTC_C64XPLUS

#ifndef NR_SOC_COMBINERS
/* By default, do not use more additional SoC combiners that the intc ones */
#define NR_SOC_COMBINERS     0
#endif
#define NR_MEGAMOD_COMBINERS 4
#define NR_COMBINERS         (NR_MEGAMOD_COMBINERS + NR_SOC_COMBINERS)

/* holds mapping of hw interrupt number to kernel IRQ number */
extern uint16_t prio_to_irq[];

#define hw_to_kernel_irq(hw)    prio_to_irq[(hw)]
#define c6x_irq_to_chip(i)      ((struct c6x_irq_chip *)irq_to_desc((i))->chip)
#define c6x_irq_desc_to_chip(d) ((struct c6x_irq_chip *)(d)->chip)

struct combiner_handler_info {
	volatile uint32_t *mevtflag;
	volatile uint32_t *evtclr;
	uint16_t irqmap[32];
};

struct combiner_mask_info {
	int irq_base;
	volatile uint32_t *evtmask;
	volatile uint32_t *evtclr;
	volatile uint32_t *evtset;
};

#define __CHIP(namestr, i, m, u)		\
    {						\
		.chip = { .name = namestr #i,	\
			  .mask = m,		\
			  .unmask = u,		\
		},				\
	}

/*
 * Functions used to map/unmap interrupts from one level to another.
 *
 * For irq_map:
 *    irq_src is a kernel IRQ number corresponding to megamodule combiner event
 *    irq_dst is a hardware interrupt number (INT4 - INT15)
 *
 * For irq_unmap:
 *    irq_src is a kernel IRQ number corresponding to megamodule combiner event
 */
extern void irq_map(unsigned int irq_src, unsigned int irq_dst);
extern void irq_unmap(unsigned int irq_src);
extern void __init init_intc_c64xplus(void);

#endif

#endif /* __ASM_C6X_IRQ_H_ */
