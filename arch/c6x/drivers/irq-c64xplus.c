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

struct combiner_handler_info {
	volatile uint32_t *mevtflag;
	volatile uint32_t *evtclr;
	uint16_t irqmap[32];
};

struct combiner_mask_info {
	int irq_base;
	volatile uint32_t *evtmask;
};

struct c6x_irq_chip {
	struct irq_chip	chip;
	struct combiner_mask_info *minfo;
};

#define c6x_irq_to_chip(i) ((struct c6x_irq_chip *)irq_to_desc((i))->chip)
#define c6x_irq_desc_to_chip(d) ((struct c6x_irq_chip *)(d)->chip)

static struct combiner_mask_info    megamod_mask_info[NR_MEGAMOD_COMBINERS];
static struct combiner_handler_info megamod_handler_info[NR_MEGAMOD_COMBINERS];

uint16_t prio_to_irq[NR_SYS_IRQS];

static uint8_t  irq_to_prio[NR_IRQS];

/* The 16 SoC GPIOs can span more than one megamodule */
#define NR_GPIO_CHIPS 2

static struct c6x_irq_chip direct_chips[(INT15 - INT4) + 1];
static struct c6x_irq_chip gpio_chips[NR_GPIO_CHIPS];
static struct c6x_irq_chip *prio_saved_chips[NR_SYS_IRQS];

#if NR_CIC_COMBINERS > 0
static struct combiner_mask_info    cic_mask_info[NR_CIC_COMBINERS];
static struct combiner_handler_info cic_handler_info[NR_CIC_COMBINERS];

static uint8_t  cic_output_to_evt[NR_CIC_OUTPUTS];
static uint8_t  cic_evt_to_output[NR_CIC_IRQS];
static struct c6x_irq_chip cic_mapped_chips[CIC_MAPLEN - NR_CIC_COMBINERS];
static struct c6x_irq_chip *cic_saved_chips[CIC_MAPLEN - NR_CIC_COMBINERS];
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

static const char *direct_chip_names[] = {
	"direct-4",
	"direct-5",
	"direct-6",
	"direct-7",
	"direct-8",
	"direct-9",
	"direct-10",
	"direct-11",
	"direct-12",
	"direct-13",
	"direct-14",
	"direct-15",
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
	struct c6x_irq_chip *chip = c6x_irq_to_chip(irq);

	irq = cic_mapped_irq(irq);
	*chip->minfo->evtmask |= (1 << (irq - chip->minfo->irq_base));
}

static void unmask_combined(unsigned int irq)
{
	struct c6x_irq_chip *chip = c6x_irq_to_chip(irq);

	irq = cic_mapped_irq(irq);
	*chip->minfo->evtmask &= ~(1 << (irq - chip->minfo->irq_base));
}

#define __CHIP(namestr, i, m, u)	\
	{				\
		.chip = { .name = namestr #i,	\
			  .mask = m,		\
			  .unmask = u,		\
		},				\
	}

/* Combiner chips */
#define MEGAMOD_CHIP(i)	\
	__CHIP("combiner-", i, mask_combined, unmask_combined)

static struct c6x_irq_chip megamod_chips[] = {
	MEGAMOD_CHIP(0),
	MEGAMOD_CHIP(1),
	MEGAMOD_CHIP(2),
	MEGAMOD_CHIP(3),
};

#if NR_CIC_COMBINERS > 0

#define CIC_CHIP(i)	\
	__CHIP("cicombiner-", i, mask_combined, unmask_combined)

static struct c6x_irq_chip cic_chips[] = {
	CIC_CHIP(0),
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
#endif /* NR_CIC_COMBINERS > 0 */

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
	struct combiner_handler_info *info = get_irq_desc_data(desc);
	unsigned long events;
	int n;

	raw_spin_lock(&desc->lock);

	if (unlikely(desc->status & IRQ_INPROGRESS))
		goto out_unlock;
	desc->status &= ~(IRQ_REPLAY | IRQ_WAITING);
	kstat_incr_irqs_this_cpu(irq, desc);

	desc->status |= IRQ_INPROGRESS;
	raw_spin_unlock(&desc->lock);

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
	struct c6x_irq_chip *chip, *direct;
	unsigned long flags;
	unsigned int prio_idx = prio - INT4;

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
			/* mask it in the megamodule combiner */
			mask_combined(cic_irq);
		} else {
			cic_src = cic_output_to_evt[output];
			if (cic_src < NR_CIC_COMBINERS)
				goto out_unlock;
			cic_irq = IRQ_CIC_START + cic_src;
		}
		desc = irq_to_desc(cic_irq);
		chip = (struct c6x_irq_chip *)desc->chip;

		direct = &direct_chips[prio_idx];
		*direct = *chip;
		direct->chip.name = direct_chip_names[prio_idx];
		direct->chip.mask = mask_direct;
		direct->chip.unmask = unmask_direct;

		prio_saved_chips[prio] = chip;
		desc->chip = (struct irq_chip *)direct;
		irq_to_prio[cic_irq] = prio;
		prio_to_irq[prio] = cic_irq;

		__irq_megamod_map(irq_src, prio);

		if (output < NR_CIC_COMBINERS)
			direct->chip.startup(cic_irq);

		goto out_unlock;
	}
#endif
	desc = irq_to_desc(irq_src);
	chip = (struct c6x_irq_chip *)desc->chip;

	direct = &direct_chips[prio_idx];
	if (irq_src < NR_MEGAMOD_COMBINERS)
		memset(&direct->chip, 0, sizeof(direct->chip));
	else
		*direct = *(struct c6x_irq_chip *)desc->chip;
	direct->chip.name = direct_chip_names[prio_idx];
	direct->chip.mask = mask_direct;
	direct->chip.unmask = unmask_direct;

	prio_saved_chips[prio] = chip;
	set_irq_chip(irq_src, (struct irq_chip *)direct);
	irq_to_prio[irq_src] = prio;
	prio_to_irq[prio] = irq_src;

	__irq_megamod_map(irq_src, prio);

	if (irq_src < NR_MEGAMOD_COMBINERS) {
		desc->action = &combiner_actions[irq_src];
		desc->handler_data = &megamod_handler_info[irq_src];
		desc->handle_irq = handle_combined_irq;
		direct->chip.startup(irq_src);
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

		desc->chip = (struct irq_chip *)prio_saved_chips[prio];

		if (output < NR_CIC_COMBINERS)
			unmask_combined(cic_irq);

		goto out_unlock;
	}
#endif
	prio = irq_to_prio[irq];
	if (!prio)
		goto out_unlock;

	desc = irq_to_desc(irq);

	and_creg(IER, ~(1 << prio));
	irq_to_prio[irq] = 0;
	prio_to_irq[prio] = IRQ_UNMAPPED;

	if (irq < NR_MEGAMOD_COMBINERS) {
		desc->chip = &dummy_irq_chip;
		desc->handle_irq = handle_bad_irq;
		desc->action = NULL;
	} else
		desc->chip = (struct irq_chip *)prio_saved_chips[prio];

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
	struct irq_desc *desc;
	struct c6x_irq_chip *chip, *mmchip;
	unsigned long flags;
	int src, output;
	struct combiner_handler_info *info;
	unsigned int idx = irq_dst - (CIC_MAPBASE + NR_CIC_COMBINERS);

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

	mmchip = &megamod_chips[(CIC_MAPBASE + output) / 32];
	info = &megamod_handler_info[(CIC_MAPBASE + output) / 32];
	info->irqmap[(CIC_MAPBASE + output) & 31] = irq_src;

	desc = irq_to_desc(irq_src);
	cic_saved_chips[idx] = (struct c6x_irq_chip *)desc->chip;

	chip = &cic_mapped_chips[idx];
	*chip = *(struct c6x_irq_chip *)desc->chip;
	chip->chip.name = mmchip->chip.name;
	chip->minfo = mmchip->minfo;

	desc->chip = (struct irq_chip *)chip;

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
	struct irq_desc *desc = irq_to_desc(irq);
	uint16_t src = irq - IRQ_CIC_START;
	uint8_t output;
	unsigned long flags;
	struct combiner_handler_info *info;
	unsigned int idx = src - NR_CIC_COMBINERS;

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

	info = &megamod_handler_info[(CIC_MAPBASE + output) / 32];
	info->irqmap[(CIC_MAPBASE + output) & 31] = CIC_MAPBASE + output;

	desc->chip = (struct irq_chip *)cic_saved_chips[idx];

out_unlock:
	spin_unlock_irqrestore(&map_lock, flags);
}
EXPORT_SYMBOL(irq_cic_unmap);

/*
 * Map a CIC source to a give CIC output event for a given core or TPCC
 */
void cic_raw_map(unsigned int src, unsigned int dst, int core)
{
	__irq_cic_map(core, src, dst);
}
EXPORT_SYMBOL(cic_raw_map);

#endif  /* NR_CIC_COMBINERS > 0 */

void __init init_pic_c64xplus(void)
{
	int i, j, idx;
	struct irq_desc *desc;
	struct c6x_irq_chip *chip, *last_chip;
	struct combiner_mask_info *minfo;
	struct combiner_handler_info *hinfo;
#if NR_CIC_COMBINERS > 0
	unsigned core = get_coreid();
#endif

	spin_lock_init(&map_lock);

	/* initialize chip info */
	minfo = &megamod_mask_info[0];
	hinfo = &megamod_handler_info[0];
	for (i = 0; i < NR_MEGAMOD_COMBINERS; i++) {
		minfo->irq_base = IRQ_EVT0 + (i * 32);
		minfo->evtmask  = &IC_EVTMASK[i];
		hinfo->mevtflag = &IC_MEVTFLAG[i];
		hinfo->evtclr   = &IC_EVTCLR[i];
		for (j = 0; j < 32; j++)
			hinfo->irqmap[j] = minfo->irq_base + j;
		minfo++;
		hinfo++;
	}
#if NR_CIC_COMBINERS > 0
	minfo = &cic_mask_info[0];
	hinfo = &cic_handler_info[0];
	for (i = 0; i < NR_CIC_COMBINERS; i++) {
		minfo->irq_base = IRQ_CIC_START + (i * 32);
		minfo->evtmask  = &CIC_EVTMASK(core)[i];
		hinfo->mevtflag = &CIC_MEVTFLAG(core)[i];
		hinfo->evtclr   = &CIC_EVTCLR(core)[i];
		for (j = 0; j < 32; j++)
			hinfo->irqmap[j] = minfo->irq_base + j;
		minfo++;
		hinfo++;
	}
#endif

	/* initialize mapping arrays */
	for (i = 0; i < NR_SYS_IRQS; i++)
		prio_to_irq[i] = IRQ_UNMAPPED;
	memset(&irq_to_prio[0], 0, sizeof(irq_to_prio));
#if NR_CIC_COMBINERS > 0
	memset(&cic_output_to_evt[0], 0, sizeof(cic_output_to_evt));
	memset(&cic_evt_to_output[0], 0, sizeof(cic_evt_to_output));

	/* megamodule IRQs coming from CIC cannot be used directly */
	for (i = CIC_MAPBASE; i < (CIC_MAPBASE + CIC_MAPLEN); i++) {
		desc = irq_to_desc(i);

		desc->status |= (IRQ_NOREQUEST | IRQ_NOPROBE);
		set_irq_chip(i, &dummy_irq_chip);
		set_irq_handler(i, handle_bad_irq);
	}

	/* CIC combined IRQs are hardwired to CIC_MAPBASE in megamodule */
	for (i = 0; i < NR_CIC_COMBINERS; i++) {
		int irq = IRQ_CIC_START + i;
		struct c6x_irq_chip *chip;

		CIC_EVTMASK(core)[i] = ~0;	/* mask all events */
		CIC_EVTCLR(core)[i] = ~0;	/* clear all events */

		cic_chips[i].minfo = &cic_mask_info[i];

		/* chip info for megamodule combiner we are wired through */
		chip = &megamod_chips[(CIC_MAPBASE + i) / 32];
		hinfo = &megamod_handler_info[(CIC_MAPBASE + i) / 32];

		/* redirect megamodule irq to right place */
		hinfo->irqmap[(CIC_MAPBASE + i) & 31] = irq;

		set_irq_chip(irq, (struct irq_chip *)chip);
		set_irq_handler(irq, handle_combined_irq);
		set_irq_data(irq, &cic_handler_info[i]);

		desc = irq_to_desc(irq);
		desc->action = &combiner_actions[NR_MEGAMOD_COMBINERS + i];
		chip->chip.startup(irq);
	}

	/* initialize individual CIC IRQs */
	for (i = NR_CIC_COMBINERS; i < (NR_CIC_COMBINERS * 32); i++) {
		int irq = IRQ_CIC_START + i;

		set_irq_chip(irq, (struct irq_chip *)&cic_chips[i / 32]);
		set_irq_handler(irq, handle_level_irq);
	}
#endif

	/*
	 * GPIO interrupts need separate copies of megamodule chips
	 * so gpio code can add edge triggering support.
	 */
	last_chip = NULL;
	idx = 0;
	for (i = IRQ_GPIO_START; i <= IRQ_GPIO15; i++) {
		chip = (struct c6x_irq_chip *)irq_to_desc(i)->chip;
		if (chip != last_chip) {
			last_chip = chip;
			gpio_chips[idx++] = *chip;
		}
		irq_to_desc(i)->chip = (struct irq_chip *)&gpio_chips[idx - 1];
	}

	/* initialize megamodule combined IRQs */
	for (i = 0; i < NR_MEGAMOD_COMBINERS; i++) {
		desc = irq_to_desc(i);

		IC_EVTMASK[i] = ~0;	/* mask all events */
		IC_EVTCLR[i] = ~0;	/* clear all events */

		desc->status |= (IRQ_NOREQUEST | IRQ_NOPROBE);
		set_irq_chip(i, &dummy_irq_chip);
		set_irq_handler(i, handle_bad_irq);

		megamod_chips[i].minfo = &megamod_mask_info[i];
		set_irq_data(i, &megamod_handler_info[i]);
	}

	/* initialize individual megamodule IRQs */
	for (i = NR_MEGAMOD_COMBINERS; i < (NR_MEGAMOD_COMBINERS * 32); i++) {
		set_irq_chip(i, (struct irq_chip *)&megamod_chips[i / 32]);
		set_irq_handler(i, handle_level_irq);
	}

	/* map megamodule combined IRQs to low-priority core IRQs */
	irq_map(IRQ_EVT0, INT12);
	irq_map(IRQ_EVT1, INT13);
	irq_map(IRQ_EVT2, INT14);
	irq_map(IRQ_EVT3, INT15);
}
