/*
 *  linux/arch/c6x/platforms/cic.c
 *
 *  Support for CIC Interrupt Controller
 *
 *  Copyright (C) 2010, 2011 Texas Instruments Incorporated
 *  Contributed by: Mark Salter <msalter@redhat.com>
 *                  Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/hardirq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/hardware.h>

struct c6x_irq_chip {
	struct irq_chip	chip;
	struct combiner_mask_info *minfo;
};

static struct combiner_mask_info     cic_mask_info[NR_CIC_COMBINERS];
static struct combiner_handler_info  cic_handler_info[NR_CIC_COMBINERS];
static uint8_t                       cic_output_to_evt[NR_CIC_OUTPUTS];
static uint8_t                       cic_evt_to_output[NR_CIC_IRQS];
static struct c6x_irq_chip           cic_mapped_chips[CIC_MAPLEN - NR_CIC_COMBINERS];
static struct c6x_irq_chip          *cic_saved_chips[CIC_MAPLEN - NR_CIC_COMBINERS];
static struct c6x_irq_chip          *megamod_chips;
static struct combiner_handler_info *megamod_handler_info;

/* lock protecting irq mappings */
static spinlock_t map_lock;

/*
 * Retrieve the corresponding mapped INTC irq for a given CIC irq (combined or not)
 * 
 * CIC IRQs mapped directly to megamodule use chip data from the megamodule
 * combiner since the megamodule combiner will controlling masking. All other
 * CIC IRQs use CIC combiner chip data.
 */
int irq_soc_mapped_irq(uint16_t irq)
{
	uint8_t output;

	/* not a CIC irq */
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

EXPORT_SYMBOL(irq_soc_mapped_irq);

static void cic_mask_combined(unsigned int irq)
{
	struct c6x_irq_chip *chip = c6x_irq_to_chip(irq);

	irq = irq_soc_mapped_irq(irq);
	*chip->minfo->evtmask |= (1 << (irq - chip->minfo->irq_base));
}

static void cic_unmask_combined(unsigned int irq)
{
	struct c6x_irq_chip *chip = c6x_irq_to_chip(irq);

	irq = irq_soc_mapped_irq(irq);
	*chip->minfo->evtmask &= ~(1 << (irq - chip->minfo->irq_base));
}

#define CIC_CHIP(i)	\
	__CHIP("cic-combiner-", i, cic_mask_combined, cic_unmask_combined)

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

/*
 * Retrieve the corresponding CIC event for a given INTC irq number
 */
int irq_soc_get(uint16_t irq_src, uint16_t *soc_irq)
{
	uint8_t  output = (irq_src - CIC_MAPBASE);
	
	if (output < NR_CIC_COMBINERS) {
		*soc_irq = IRQ_CIC_START + output;
		return 1;
	} else {
		uint8_t cic_src = cic_output_to_evt[output];
		if (cic_src < NR_CIC_COMBINERS)
			return -1;
		*soc_irq = IRQ_CIC_START + cic_src;
		return 0;
	}
}
EXPORT_SYMBOL(irq_soc_get);

int irq_soc_init(void)
{
	int i, j;
	struct combiner_mask_info *minfo;
	struct combiner_handler_info *hinfo;
	unsigned core = get_coreid();

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

	memset(&cic_output_to_evt[0], 0, sizeof(cic_output_to_evt));
	memset(&cic_evt_to_output[0], 0, sizeof(cic_evt_to_output));

	return 0;
}
EXPORT_SYMBOL(irq_soc_init);

int irq_soc_setup(struct c6x_irq_chip          *parent_chips,
		  struct combiner_handler_info *parent_handler_info,
		  void (*handler)(unsigned int irq, struct irq_desc *desc),
		  struct irqaction             *combiner_actions)
{
	int i;
	struct irq_desc *desc;
	struct combiner_handler_info *hinfo;
	unsigned core = get_coreid();

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
		CIC_EVTCLR(core)[i]  = ~0;	/* clear all events */

		cic_chips[i].minfo = &cic_mask_info[i];

		/* chip info for parent combiner we are wired through */
		chip  = &parent_chips[(CIC_MAPBASE + i) / 32];
		hinfo = &parent_handler_info[(CIC_MAPBASE + i) / 32];

		/* redirect parent irq to right place */
		hinfo->irqmap[(CIC_MAPBASE + i) & 31] = irq;

		set_irq_chip(irq, (struct irq_chip *)chip);
		set_irq_handler(irq, handler);
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

	megamod_chips = parent_chips;
	megamod_handler_info = parent_handler_info;

	spin_lock_init(&map_lock);

	return 0;
}
EXPORT_SYMBOL(irq_soc_setup);

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
 * For irq_cic_map:
 *    irq_src is a kernel IRQ number corresponding to CIC combiner event
 *    irq_dst is a kernel IRQ number corresponding to megamodule combiner event
 *
 * For irq_cic_unmap:
 *    irq_src is a kernel IRQ number corresponding to CIC combiner event
 *
 *
 * In order to map a CIC event directly to a core hardware interrupt, it must
 * first be mapped to a megamodule event with irq_cic_map(). Then the megamodule
 * event can be mapped to a core hardware interrupt with irq_map(). To unmap,
 * first unmap the megamodule event, then the CIC event.
 */

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
