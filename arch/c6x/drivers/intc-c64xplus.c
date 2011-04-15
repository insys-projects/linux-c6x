/*
 *  linux/arch/c6x/drivers/irq-c64xplus.c
 *
 *  Support for C64x+ Megamodule Interrupt Controller
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Contributed by: Mark Salter <msalter@redhat.com>
 *                  Aurelien Jacquiot <a-jacquiot@ti.com>
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

#define IRQ_UNMAPPED         0xffff

static struct combiner_mask_info    megamod_mask_info[NR_MEGAMOD_COMBINERS];
static struct combiner_handler_info megamod_handler_info[NR_MEGAMOD_COMBINERS];

uint16_t       prio_to_irq[NR_SYS_IRQS];
static uint8_t irq_to_prio[NR_IRQS];

/* The 16 SoC GPIOs can span more than one megamodule */
#define NR_GPIO_CHIPS 2

struct c6x_irq_chip {
	struct irq_chip	chip;
	struct combiner_mask_info *minfo;
};

static struct c6x_irq_chip direct_chips[(INT15 - INT4) + 1];
static struct c6x_irq_chip gpio_chips[NR_GPIO_CHIPS];
static struct c6x_irq_chip *prio_saved_chips[NR_SYS_IRQS];

#if NR_SOC_COMBINERS > 0
extern int irq_soc_mapped_irq(uint16_t irq);
extern int irq_soc_get(uint16_t irq_src, uint16_t *soc_irq);
extern int irq_soc_init(void);
extern int irq_soc_setup(struct c6x_irq_chip          *parent_chips,
			 struct combiner_handler_info *parent_handler_info,
			 void (*handler)(unsigned int irq, struct irq_desc *desc),
			 struct irqaction             *combiner_actions);
#else /* NR_SOC_COMBINERS > 0 */
#define irq_soc_mapped_irq(i) (i)
#endif /* NR_SOC_COMBINERS > 0 */

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

static void mask_combined(unsigned int irq)
{
	struct c6x_irq_chip *chip = c6x_irq_to_chip(irq);

	irq = irq_soc_mapped_irq(irq);
	if (likely(chip->minfo->evtmask))
		*chip->minfo->evtmask |= (1 << (irq - chip->minfo->irq_base));
}

static void unmask_combined(unsigned int irq)
{
	struct c6x_irq_chip *chip = c6x_irq_to_chip(irq);

	irq = irq_soc_mapped_irq(irq);
	if (likely(chip->minfo->evtmask))
		*chip->minfo->evtmask &= ~(1 << (irq - chip->minfo->irq_base));
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

static struct irqaction combiner_actions[NR_MEGAMOD_COMBINERS + NR_SOC_COMBINERS];
static char combiner_actions_name[NR_MEGAMOD_COMBINERS + NR_SOC_COMBINERS][20];

#if NR_SOC_COMBINERS > 0
#define PRE_ACK(irq) IRQ_SOC_COMBINER_PRE_ACK(irq)
#else
/* For IRQs through the megamodule, ack before handler runs */
#define PRE_ACK(irq) 1
#endif

static 
void handle_combined_irq(unsigned int irq, struct irq_desc *desc)
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

		if (PRE_ACK(irq))
			*info->evtclr = (1 << n);

		generic_handle_irq(irq);

		if (!PRE_ACK(irq))
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
	val = INTC_INTMUX[dst >> 2];
	val &= ~((nr_srcs - 1) << offset);
	val |= ((src & (nr_srcs - 1)) << offset);
	INTC_INTMUX[dst >> 2] = val;
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

#if NR_SOC_COMBINERS > 0
	/* handle mapping for IRQs coming from SoC INTC */
	if (IRQ_SOC_COMBINER(irq_src)) {
		uint16_t soc_irq;
		int      res;
		res = irq_soc_get(irq_src, &soc_irq);
		if (res < 0)
			goto out_unlock;
		if (res)
			/* mask it in the megamodule combiner */
			mask_combined(soc_irq);

		desc = irq_to_desc(soc_irq);
		chip = (struct c6x_irq_chip *)desc->chip;

		direct = &direct_chips[prio_idx];
		*direct = *chip;
		direct->chip.name = direct_chip_names[prio_idx];
		direct->chip.mask = mask_direct;
		direct->chip.unmask = unmask_direct;

		prio_saved_chips[prio] = chip;
		desc->chip = (struct irq_chip *)direct;
		irq_to_prio[soc_irq] = prio;
		prio_to_irq[prio] = soc_irq;

		__irq_megamod_map(irq_src, prio);

		if (IRQ_SOC_COMBINER_COMBINED(irq_src))
			direct->chip.startup(soc_irq);

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

#if NR_SOC_COMBINERS > 0
	if (IRQ_SOC_COMBINER(irq)) {
		uint16_t soc_irq;
		int      res;

		prio = 0;

		res = irq_soc_get(irq, &soc_irq);
		if (res < 0)
			goto out_unlock;

		desc = irq_to_desc(soc_irq);

		prio = irq_to_prio[soc_irq];
		if (!prio)
			goto out_unlock;

		mask_direct(soc_irq);
		irq_to_prio[soc_irq] = 0;
		prio_to_irq[prio] = IRQ_UNMAPPED;

		desc->chip = (struct irq_chip *)prio_saved_chips[prio];

		if (IRQ_SOC_COMBINER_COMBINED(irq))
			unmask_combined(soc_irq);

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


void __init init_intc_c64xplus(void)
{
	int i, j, idx;
	struct irq_desc *desc;
	struct c6x_irq_chip *chip, *last_chip;
	struct combiner_mask_info *minfo;
	struct combiner_handler_info *hinfo;
	unsigned int nr_combiners;

	spin_lock_init(&map_lock);

	/* initialize combiner actions based on the total amount of combiners  */
	nr_combiners = NR_MEGAMOD_COMBINERS + NR_SOC_COMBINERS;
	for (i = 0; i < nr_combiners; i++) {
		combiner_actions[i].name = &combiner_actions_name[i][0];
		sprintf((char *) combiner_actions[i].name,
			"combined-%d-%d", i << 5, (i << 5) + 32 - 1);
	}

	/* initialize chip info */
	minfo = &megamod_mask_info[0];
	hinfo = &megamod_handler_info[0];
	for (i = 0; i < NR_MEGAMOD_COMBINERS; i++) {
		minfo->irq_base = IRQ_EVT0 + (i * 32);
		minfo->evtmask  = &INTC_EVTMASK[i];
		hinfo->mevtflag = &INTC_MEVTFLAG[i];
		hinfo->evtclr   = &INTC_EVTCLR[i];
		for (j = 0; j < 32; j++)
			hinfo->irqmap[j] = minfo->irq_base + j;
		minfo++;
		hinfo++;
	}

#if NR_SOC_COMBINERS > 0
	irq_soc_init();
#endif

	/* initialize mapping arrays */
	for (i = 0; i < NR_SYS_IRQS; i++)
		prio_to_irq[i] = IRQ_UNMAPPED;
	memset(&irq_to_prio[0], 0, sizeof(irq_to_prio));

	/* initialize megamodule combined IRQs */
	for (i = 0; i < NR_MEGAMOD_COMBINERS; i++) {
		desc = irq_to_desc(i);

		INTC_EVTMASK[i] = ~0;	/* mask all events */
		INTC_EVTCLR[i]  = ~0;	/* clear all events */

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

#if NR_SOC_COMBINERS > 0
	irq_soc_setup(megamod_chips,
		      megamod_handler_info,
		      handle_combined_irq,
		      combiner_actions);
#endif

	/* 
	 * Clear again megamodule combined IRQs because spurious interrupts from 
	 * SoC combiner may have occur after the SoC combiner initialization.
	 */
	for (i = 0; i < NR_MEGAMOD_COMBINERS; i++)
		INTC_EVTCLR[i] = ~0; /* clear all events */

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

	/* map megamodule combined IRQs to low-priority core IRQs */
	irq_map(IRQ_EVT0, INT12);
	irq_map(IRQ_EVT1, INT13);
	irq_map(IRQ_EVT2, INT14);
	irq_map(IRQ_EVT3, INT15);
}
