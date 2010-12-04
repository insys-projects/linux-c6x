/*
 *  linux/arch/c6x/drivers/irq-c64xplus.c
 *
 *  Support for C64x+ Megamodule Interrupt Controller
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Contributed by: Mark Salter <msalter@redhat.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kernel_stat.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/hardirq.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/traps.h>
#include <asm/page.h>
#include <asm/machdep.h>
#include <asm/hardirq.h>
#include <asm/hardware.h>

#define IRQ_UNMAPPED 0xffff

#define NR_MEGAMOD_COMBINERS 4

#ifdef CONFIG_SOC_TMS320C6474
#define NR_CIC_COMBINERS 2
#define NR_CIC_OUTPUTS   16
#else
#define NR_CIC_COMBINERS 0
#endif

#define NR_COMBINERS (NR_MEGAMOD_COMBINERS + NR_CIC_COMBINERS)

/* combiner info used by flow handlers and combiner chips */
struct combiner_info {
	int irq_base;
	volatile uint32_t *evtmask;
	volatile uint32_t *mevtflag;
	volatile uint32_t *evtclr;
	uint16_t irqmap[32];
};

static struct combiner_info chip_info[NR_COMBINERS];

uint16_t prio_to_irq[NR_SYS_IRQS];

static uint8_t  irq_to_prio[NR_IRQS];

static struct irq_chip *prio_saved_chip[NR_SYS_IRQS];

#if NR_CIC_COMBINERS > 0
static uint8_t  cic_output_to_evt[NR_CIC_OUTPUTS];
static uint8_t  cic_evt_to_output[NR_CIC_IRQS];
#endif

/* lock protecting irq mappings */
static spinlock_t map_lock;

static void mask_direct(unsigned int irq)
{
	uint16_t prio;

	BUG_ON(irq >= NR_IRQS);
	prio = irq_to_prio[irq];
	BUG_ON(prio >= NR_SYS_IRQS);
	disable_irq_mask(1 << prio);
}

static void unmask_direct(unsigned int irq)
{
	uint16_t prio;

	BUG_ON(irq >= NR_IRQS);
	prio = irq_to_prio[irq];
	BUG_ON(prio >= NR_SYS_IRQS);
	enable_irq_mask(1 << prio);
}

#define __CHIP(namestr, i, m, u)	\
	{				\
		.name = namestr #i,	\
		.mask = m,		\
		.unmask = u,		\
	}
/* Direct HW interrupt chip */
#define DIRECT_CHIP(i)			\
	__CHIP("direct-", i, mask_direct, unmask_direct)

static struct irq_chip direct_chips[] = {
	DIRECT_CHIP(0),	 DIRECT_CHIP(1),
	DIRECT_CHIP(2),	 DIRECT_CHIP(3),
	DIRECT_CHIP(4),	 DIRECT_CHIP(5),
	DIRECT_CHIP(6),	 DIRECT_CHIP(7),
	DIRECT_CHIP(8),	 DIRECT_CHIP(9),
	DIRECT_CHIP(10), DIRECT_CHIP(11),
	DIRECT_CHIP(12), DIRECT_CHIP(13),
	DIRECT_CHIP(14), DIRECT_CHIP(15),
};


#if NR_CIC_COMBINERS > 0
/*
 * CIC IRQs mapped directly to megamodule use chip data from the megamodule
 * combiner since the megamodule combiner will controlling masking. All other
 * CIC IRQs use CIC combiner chip data.
 */
static inline int cic_mapped_irq(uint16_t irq)
{
	uint8_t output;

	if (irq < IRQ_CIC_START || irq >= (IRQ_CIC_START + NR_CIC_IRQS))
		return irq;

	/* the CIC combined IRQs are hardwired */
	if (irq < (IRQ_CIC_START + NR_CIC_COMBINERS))
		return CIC_MAPBASE + (irq - IRQ_CIC_START);

	/* the other CIC events may or may not be mapped to megamodule */
	output = cic_evt_to_output[irq - IRQ_CIC_START];
	if (output)
		return CIC_MAPBASE + output;

	return irq;
}
#else
#define cic_mapped_irq(i) (i)
#endif

static void mask_combined(unsigned int irq)
{
	struct combiner_info *info = get_irq_chip_data(irq);

	irq = cic_mapped_irq(irq);
	*info->evtmask |= (1 << ((irq - info->irq_base) & 31));
}

static void unmask_combined(unsigned int irq)
{
	struct combiner_info *info = get_irq_chip_data(irq);

	irq = cic_mapped_irq(irq);
	*info->evtmask &= ~(1 << ((irq - info->irq_base) & 31));
}

/* Combiner chips */
#define MEGAMOD_CHIP(i)	\
	__CHIP("combiner-", i, mask_combined, unmask_combined)

#define CIC_CHIP(i)	\
	__CHIP("cicombiner-", i, mask_combined, unmask_combined)

static struct irq_chip combiner_chips[] = {
	MEGAMOD_CHIP(0), MEGAMOD_CHIP(1),
	MEGAMOD_CHIP(2), MEGAMOD_CHIP(3),
#if NR_CIC_COMBINERS > 0
	CIC_CHIP(0),
#endif
#if NR_CIC_COMBINERS > 1
	CIC_CHIP(1),
#endif
#if NR_CIC_COMBINERS > 2
	CIC_CHIP(2),
#endif
#if NR_CIC_COMBINERS > 3
	CIC_CHIP(3),
#endif
};

static struct irqaction combiner_actions[] = {
	{ .name = "combined-0-31", },
	{ .name = "combined-32-63", },
	{ .name = "combined-64-95", },
	{ .name = "combined-96-127", },
#if NR_CIC_COMBINERS > 0
	{ .name = "combined-128-159", },
#endif
#if NR_CIC_COMBINERS > 1
	{ .name = "combined-160-191", },
#endif
};

/*
 * When handling IRQs through the CIC, ack after the handler runs.
 * For IRQs through the megamodule, ack before handler runs.
 */
#if NR_CIC_COMBINERS
#define PRE_ACK (irq < IRQ_CIC_START || \
		 irq >= (IRQ_CIC_START + NR_CIC_COMBINERS))
#else
#define PRE_ACK 1
#endif

static void handle_combined_irq(unsigned int irq, struct irq_desc *desc)
{
	struct combiner_info *info;
	unsigned long events;
	int n;

	raw_spin_lock(&desc->lock);

	if (unlikely(desc->status & IRQ_INPROGRESS))
		goto out_unlock;
	desc->status &= ~(IRQ_REPLAY | IRQ_WAITING);
	kstat_incr_irqs_this_cpu(irq, desc);

	desc->status |= IRQ_INPROGRESS;
	raw_spin_unlock(&desc->lock);

	info = get_irq_desc_data(desc);
	while ((events = *info->mevtflag) != 0) {
		n = __ffs(events);
		irq = info->irqmap[n]; /* irq to handle */

		if (PRE_ACK)
			*info->evtclr = (1 << n);

		generic_handle_irq(irq);

		if (!PRE_ACK)
			*info->evtclr = (1 << n);
	}

	raw_spin_lock(&desc->lock);
	desc->status &= ~IRQ_INPROGRESS;
out_unlock:
	raw_spin_unlock(&desc->lock);
}


/*
 * Setup megamodule event mappings.
 */
static inline void __irq_megamod_map(unsigned int src, unsigned int dst)
{
	uint32_t val;
	int offset, nr_srcs;

	nr_srcs = (NR_MEGAMOD_COMBINERS * 32);

	BUG_ON(dst < INT4 || dst > INT15);
	BUG_ON(src >= nr_srcs);

	/* each mux register controls four outputs */
	offset = (dst & 3) << 3;
	val = IC_INTMUX[dst >> 2];
	val &= ~((nr_srcs - 1) << offset);
	val |= ((src & (nr_srcs - 1)) << offset);
	IC_INTMUX[dst >> 2] = val;
}

/*
 * Map a C64x+ megamodule event to a CPU core priority interrupt.
 */
void irq_map(unsigned int irq_src, unsigned int prio)
{
	struct irq_desc *desc;
	unsigned long flags;

	if (prio < INT4 || prio > INT15)
		return;

	/* only map megamodule event sources here */
	if (irq_src >= (NR_MEGAMOD_COMBINERS * 32))
		return;

	spin_lock_irqsave(&map_lock, flags);

	/* make sure this IRQ is not mapped to another core IRQ */
	if (irq_to_prio[irq_src])
		goto out_unlock;

	/* make sure no other IRQ is mapped to the requested priority */
	if (prio_to_irq[prio] != IRQ_UNMAPPED)
		goto out_unlock;

#if NR_CIC_COMBINERS > 0
	/* handle mapping for IRQs coming from CIC */
	if (irq_src >= CIC_MAPBASE &&
	    irq_src < (CIC_MAPBASE + CIC_MAPLEN)) {
		uint8_t cic_src, output = (irq_src - CIC_MAPBASE);
		uint16_t cic_irq;

		if (output < NR_CIC_COMBINERS) {
			cic_irq = IRQ_CIC_START + output;

			mask_combined(cic_irq);
		} else {
			cic_src = cic_output_to_evt[output];
			if (cic_src < NR_CIC_COMBINERS)
				goto out_unlock;
			cic_irq = IRQ_CIC_START + cic_src;
		}
		desc = irq_to_desc(cic_irq);
		prio_saved_chip[prio] = desc->chip;
		set_irq_chip(cic_irq, &direct_chips[prio]);

		irq_to_prio[cic_irq] = prio;
		prio_to_irq[prio] = cic_irq;

		__irq_megamod_map(irq_src, prio);

		if (output < NR_CIC_COMBINERS)
			desc->chip->startup(cic_irq);

		goto out_unlock;
	}
#endif
	/* record the mapping */
	irq_to_prio[irq_src] = prio;
	prio_to_irq[prio] = irq_src;

	desc = irq_to_desc(irq_src);

	prio_saved_chip[prio] = desc->chip;
	set_irq_chip(irq_src, &direct_chips[prio]);

	__irq_megamod_map(irq_src, prio);

	if (irq_src < NR_MEGAMOD_COMBINERS) {
		desc->handler_data = &chip_info[irq_src];
		desc->handle_irq = handle_combined_irq;
		desc->chip->startup(irq_src);
		/* only so it shows up in /proc */
		desc->action = &combiner_actions[irq_src];
	}

out_unlock:
	spin_unlock_irqrestore(&map_lock, flags);
}
EXPORT_SYMBOL(irq_map);

/*
 * unmap a C64x+ interrupt source from a direct vector back to an IRQ combiner.
 */
void irq_unmap(unsigned int irq)
{
	struct irq_desc *desc;
	unsigned int prio;
	unsigned long flags;

	/* only unmap megamodule event sources */
	if (irq >= (NR_MEGAMOD_COMBINERS * 32))
		return;

	spin_lock_irqsave(&map_lock, flags);

#if NR_CIC_COMBINERS > 0
	if (irq >= CIC_MAPBASE && irq < (CIC_MAPBASE + CIC_MAPLEN)) {
		uint8_t output = irq - CIC_MAPBASE;
		uint16_t cic_irq;

		prio = 0;
		if (output < NR_CIC_COMBINERS)
			cic_irq = IRQ_CIC_START + output;
		else {
			cic_irq = cic_output_to_evt[output];
			if (!cic_irq)
				goto out_unlock;
			cic_irq += IRQ_CIC_START;
		}
		desc = irq_to_desc(cic_irq);

		prio = irq_to_prio[cic_irq];
		if (!prio)
			goto out_unlock;

		mask_direct(cic_irq);
		irq_to_prio[cic_irq] = 0;
		prio_to_irq[prio] = IRQ_UNMAPPED;

		set_irq_chip(cic_irq, prio_saved_chip[prio]);

		if (output < NR_CIC_COMBINERS)
			unmask_combined(cic_irq);
	} else
#endif
	{
		prio = irq_to_prio[irq];
		if (!prio)
			goto out_unlock;

		desc = irq_to_desc(irq);

		and_creg(IER, ~(1 << prio));
		irq_to_prio[irq] = 0;
		prio_to_irq[prio] = IRQ_UNMAPPED;

		if (irq < NR_MEGAMOD_COMBINERS) {
			set_irq_chip(irq, &dummy_irq_chip);
			desc->handle_irq = handle_bad_irq;
			desc->action = NULL;
		} else
			set_irq_chip(irq, prio_saved_chip[prio]);
	}

out_unlock:
	spin_unlock_irqrestore(&map_lock, flags);
}
EXPORT_SYMBOL(irq_unmap);

#if NR_CIC_COMBINERS > 0
/*
 * Setup CIC event mappings on given core.
 */
static inline void __irq_cic_map(int core, unsigned int src, unsigned int dst)
{
	uint32_t val;
	int offset, nr_srcs;

	nr_srcs = (NR_CIC_COMBINERS * 32);

	/* output0 and output1 are hardwired */
	BUG_ON(dst < 2 || dst > 15);
	BUG_ON(src >= nr_srcs);

	offset = (dst & 3) << 3;
	val = CIC_MUX(core)[dst >> 2];
	val &= ~(0x7f << offset);
	val |= ((src & 0x7f) << offset);
	CIC_MUX(core)[dst >> 2] = val;
}

/*
 * Map a C64x+ CIC IRQ to a megamodule event
 *
 * Do nothing for CIC combined IRQs. They are always mapped.
 */
void irq_cic_map(unsigned int irq_src, unsigned int irq_dst)
{
	unsigned long flags;
	int src, output;
	struct combiner_info *info;

	if (irq_dst < (CIC_MAPBASE + NR_CIC_COMBINERS) ||
	    irq_dst >= (CIC_MAPBASE + CIC_MAPLEN))
		return;

	/* only map CIC event sources */
	if (irq_src < (IRQ_CIC_START + NR_CIC_COMBINERS) ||
	    irq_src >= (IRQ_CIC_START + NR_CIC_IRQS))
		return;

	src = irq_src - IRQ_CIC_START;
	output = irq_dst - CIC_MAPBASE;

	spin_lock_irqsave(&map_lock, flags);

	/* make sure this IRQ is not already mapped */
	if (cic_evt_to_output[src])
		goto out_unlock;

	/* make sure the requested output is not in use */
	if (cic_output_to_evt[output])
		goto out_unlock;

	/* record the mapping */
	cic_evt_to_output[src] = output;
	cic_output_to_evt[output] = src;

	info = &chip_info[(CIC_MAPBASE + output) / 32];
	info->irqmap[(CIC_MAPBASE + output) & 31] = irq_src;

	set_irq_chip(irq_src, &combiner_chips[(CIC_MAPBASE + output) / 32]);
	set_irq_chip_data(irq_src, info);

	__irq_cic_map(get_coreid(), src, output);
out_unlock:
	spin_unlock_irqrestore(&map_lock, flags);
}
EXPORT_SYMBOL(irq_cic_map);

/*
 * unmap a C64x+ CIC interrupt source from a megamodule IRQ
 */
void irq_cic_unmap(unsigned int irq)
{
	uint16_t src = irq - IRQ_CIC_START;
	uint8_t output;
	unsigned long flags;
	struct combiner_info *info;

	/* only unmap cic event sources */
	if (irq < (IRQ_CIC_START + NR_CIC_COMBINERS) ||
	    irq >= (IRQ_CIC_START + NR_CIC_IRQS))
		return;

	spin_lock_irqsave(&map_lock, flags);

	output = cic_evt_to_output[src];
	if (output < NR_CIC_COMBINERS)
		goto out_unlock;

	/* undo the mapping */
	cic_evt_to_output[src] = 0;
	cic_output_to_evt[output] = 0;

	info = get_irq_chip_data(irq);
	info->irqmap[(CIC_MAPBASE + output) & 31] = CIC_MAPBASE + output;

	set_irq_chip(irq, &combiner_chips[NR_MEGAMOD_COMBINERS + (src / 32)]);
	set_irq_chip_data(irq, &chip_info[NR_MEGAMOD_COMBINERS + (src / 32)]);

out_unlock:
	spin_unlock_irqrestore(&map_lock, flags);
}
EXPORT_SYMBOL(irq_cic_unmap);
#endif  /* NR_CIC_COMBINERS > 0 */


void __init init_pic_c64xplus(void)
{
	int i, j;
	struct combiner_info *info = &chip_info[0];
#if NR_CIC_COMBINERS > 0
	unsigned core = get_coreid();
#endif

	spin_lock_init(&map_lock);

	/* initialize chip info */
	for (i = 0; i < NR_MEGAMOD_COMBINERS; i++) {
		info->irq_base = IRQ_EVT0 + (i * 32);
		info->mevtflag = &IC_MEVTFLAG[i];
		info->evtclr   = &IC_EVTCLR[i];
		info->evtmask  = &IC_EVTMASK[i];
		for (j = 0; j < 32; j++)
			info->irqmap[j] = info->irq_base + j;
		info++;
	}
#if NR_CIC_COMBINERS > 0
	for (i = 0; i < NR_CIC_COMBINERS; i++) {
		info->irq_base = IRQ_CIC_START + (i * 32);
		info->mevtflag = &CIC_MEVTFLAG(core)[i];
		info->evtclr   = &CIC_EVTCLR(core)[i];
		info->evtmask  = &CIC_EVTMASK(core)[i];
		for (j = 0; j < 32; j++)
			info->irqmap[j] = info->irq_base + j;
		info++;
	}
#endif

	/* initialize mapping arrays */
	for (i = 0; i < NR_SYS_IRQS; i++)
		prio_to_irq[i] = IRQ_UNMAPPED;
	memset(&irq_to_prio[0], 0, sizeof(irq_to_prio));
#if NR_CIC_COMBINERS > 0
	memset(&cic_output_to_evt[0], 0, sizeof(cic_output_to_evt));
	memset(&cic_evt_to_output[0], 0, sizeof(cic_evt_to_output));
#endif

	/* initialize megamodule combined IRQs */
	for (i = 0; i < NR_MEGAMOD_COMBINERS; i++) {
		struct irq_desc *desc = irq_to_desc(i);

		IC_EVTMASK[i] = ~0;	/* mask all events */
		IC_EVTCLR[i] = ~0;	/* clear all events */

		desc->status |= (IRQ_NOREQUEST | IRQ_NOPROBE);
		set_irq_chip(i, &dummy_irq_chip);
		set_irq_handler(i, handle_bad_irq);
	}

	/* initialize individual megamodule IRQs */
	for (i = NR_MEGAMOD_COMBINERS; i < (NR_MEGAMOD_COMBINERS * 32); i++) {
		set_irq_chip(i, &combiner_chips[i / 32]);
		set_irq_chip_data(i, &chip_info[i / 32]);
		set_irq_handler(i, handle_level_irq);
	}

#if NR_CIC_COMBINERS > 0
	/* megamodule IRQs coming from CIC cannot be used directly */
	for (i = CIC_MAPBASE; i < (CIC_MAPBASE + CIC_MAPLEN); i++) {
		struct irq_desc *desc = irq_to_desc(i);

		desc->status |= (IRQ_NOREQUEST | IRQ_NOPROBE);
		set_irq_chip(i, &dummy_irq_chip);
		set_irq_handler(i, handle_bad_irq);
	}

	/* CIC combined IRQs are hardwired to CIC_MAPBASE in megamodule */
	for (i = 0; i < NR_CIC_COMBINERS; i++) {
		struct irq_desc *desc = irq_to_desc(IRQ_CIC_START + i);
		struct combiner_info *info;

		CIC_EVTMASK(core)[i] = ~0;	/* mask all events */
		CIC_EVTCLR(core)[i] = ~0;	/* clear all events */

		/* chip info for megamodule combiner we are wired through */
		info = &chip_info[(CIC_MAPBASE + i) / 32];

		set_irq_chip(IRQ_CIC_START + i,
			     &combiner_chips[(CIC_MAPBASE + i) / 32]);
		desc->chip_data = info;
		desc->handle_irq = handle_combined_irq;
		desc->handler_data = &chip_info[NR_MEGAMOD_COMBINERS + i];
		desc->action = &combiner_actions[NR_MEGAMOD_COMBINERS + i];

		desc->chip->startup(IRQ_CIC_START + i);

		/* redirect megamodule irq to right place */
		info->irqmap[(CIC_MAPBASE + i) & 31] = IRQ_CIC_START + i;
	}

	/* initialize individual CIC IRQs */
	for (i = NR_CIC_COMBINERS; i < (NR_CIC_COMBINERS * 32); i++) {
		struct irq_desc *desc = irq_to_desc(IRQ_CIC_START + i);

		set_irq_chip(desc->irq,
			     &combiner_chips[NR_MEGAMOD_COMBINERS + (i / 32)]);
		desc->chip_data = &chip_info[NR_MEGAMOD_COMBINERS + (i / 32)];
		desc->handle_irq = handle_level_irq;
	}
#endif

	/* map megamodule combined IRQs to low-priority core IRQs */
	irq_map(IRQ_EVT0, INT12);
	irq_map(IRQ_EVT1, INT13);
	irq_map(IRQ_EVT2, INT14);
	irq_map(IRQ_EVT3, INT15);
}
