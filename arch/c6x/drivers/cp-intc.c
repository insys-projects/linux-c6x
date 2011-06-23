/*
 *  linux/arch/c6x/drivers/cp-intc.c
 *
 *  Support for C66x Communication Port Interrupt Controller (CP_INTC)
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated
 *  Contributed by: Aurelien Jacquiot <a-jacquiot@ti.com>
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
#include <asm/cp-intc.h>

#ifdef DEBUG
#define DPRINTK(fmt, args...) printk(KERN_CRIT "INTC: [%s] " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...) 
#endif

struct c6x_irq_chip {
	struct irq_chip	chip;
	struct combiner_mask_info *minfo;
};

static struct combiner_mask_info     cpintc_mask_info[NR_CPINTC0_COMBINERS];
static struct combiner_handler_info  cpintc_handler_info[NR_CPINTC0_COMBINERS];
static uint8_t                       cpintc_evt_to_output[NR_CPINTC0_IRQS];
static struct c6x_irq_chip           cpintc_mapped_chips[NR_CPINTC0_OUTPUTS];
static struct c6x_irq_chip          *megamod_chips;
static struct combiner_handler_info *megamod_handler_info;

#define __host_irq_to_idx(irq) IRQ_SOC_HOST_IRQ_TO_IDX(irq)
#define __idx_to_chan(i)       IRQ_SOC_IDX_TO_CHAN(i)

/*
 * There is one CP_INTC for 4 cores on C66x
 */
#if (CORE_NUM < 5)
#define __get_cpintc_id()      0
#else
#define __get_cpintc_id()      (get_coreid() >> 2)
#endif

/* Mapping from host int to INTC mapping */
static uint8_t cpintc_host_irq_map[] = { 
	IRQ_INTC0OUT,
	IRQ_INTC0OUT + 1,
	IRQ_INTC0OUT + 2,
	IRQ_INTC0OUT + 3,
	IRQ_INTC0OUT + 4,
	IRQ_INTC0OUT + 5,
	IRQ_INTC0OUT + 6,
	IRQ_INTC0OUT + 7,
};

/* Lock protecting irq mappings */
static spinlock_t map_lock;

/*
 *  Return 1 if the INTC IRQ number is a CP_INTC interrupt
 */
int cpintc_irq(unsigned int irq)
{
	int i;

	/* Scan host IRQ */
	for (i = 0; i < IRQ_CPINTC0_MAPLEN; i++) {
		if (irq == cpintc_host_irq_map[i])
			return 1;
	}
	return 0;
}
EXPORT_SYMBOL(cpintc_irq);

/*
 *  Return 1 if the INTC IRQ number is a CP_INTC combined interrupt
 */
int cpintc_combined_irq(unsigned int irq)
{
	int i;

	/* The combined interrupt are hardcoded at the beginning */
	for (i = 0; i < NR_CPINTC0_COMBINERS; i++) {
		if (irq == cpintc_host_irq_map[i])
			return 1;
	}
	return 0;
}
EXPORT_SYMBOL(cpintc_combined_irq);

/*
 * Map a given CP_INTC source (system interrupt) to a given CP_INTC channel
 */
static inline void __cpintc_map_irq(int n, unsigned int src, unsigned int channel)
{
	uint32_t val;
	int      offset;

	BUG_ON(src >= NR_CPINTC0_COMBINERS * 32);
	BUG_ON(channel >= NR_CPINTC0_CHANNELS);

	if (src >= NR_CPINTC0_IRQS)
	    return;

	offset = (src & 3) << 3;

	val  = CPINTC_CHMAP(n)[src >> 2];
	val &= ~(0xff << offset);
	val |= ((channel & 0xff) << offset);

	CPINTC_CHMAP(n)[src >> 2] = val;

	DPRINTK("mapping src %d IRQ to channel %d IRQ, *CPINTC_CHMAP (0x%x) = 0x%x\n",
		src, channel, &CPINTC_CHMAP(n)[src >> 2], CPINTC_CHMAP(n)[src >> 2]);
}

/*
 * Retrieve the channel for a given host irq
 * -1 is returned if the host irq is not mapped
 */
static inline int __cpintc_get_host_irq(int n, unsigned int dst)
{
	unsigned int channel;
	uint32_t     val;
	int          offset;

	for (channel = 0; channel < NR_CPINTC0_CHANNELS; channel++) {
		offset = (channel & 3) << 3;
		val    = CPINTC_HINTMAP(n)[channel >> 2];
		val   &= (0xff << offset);
		if (val == ((dst & 0xff) << offset)) {
		    	DPRINTK("channel for host irq %d is %d\n", dst, channel);
			return (int) channel;
		}
	}
	
	DPRINTK("channel for host irq %d not found\n", dst);
	return -1;
}

/*
 * Map a given CP_INTC channel to a particular CP_INTC host interrupt
 */
static inline void __cpintc_map_channel(int n,
					unsigned int channel,
					unsigned int dst)
{
	uint32_t val;
	int      offset;

	BUG_ON(channel >= NR_CPINTC0_CHANNELS);

	offset = (channel & 3) << 3;

	val  = CPINTC_HINTMAP(n)[channel >> 2];
	val &= ~(0xff << offset);
	val |= ((dst & 0xff) << offset);

	CPINTC_HINTMAP(n)[channel >> 2] = val;
}

/*
 * Map a CP_INTC source to a given INTC event
 */
int cpintc_raw_map(int n, unsigned int src, unsigned int dst)
{
	int channel;

	channel = __cpintc_get_host_irq(n, dst);
	if (channel == -1) {
		/* Map it (if SoC allows it) */
		__cpintc_map_channel(n, channel, dst);
		channel = __cpintc_get_host_irq(n, dst);
		if (channel == -1)
			/* Did not succeed to map it */
			return -1;
	}
 	__cpintc_map_irq(n, src, channel);
	return 0;
}
EXPORT_SYMBOL(cpintc_raw_map);

/*
 * Map a CP_INTC event to a megamodule event
 * 
 * irq_src is a kernel IRQ number corresponding to CP_INTC input event (system interrupt)
 * irq_dst is a kernel IRQ number corresponding to a megamodule event (host interrupt)
 *
 * In order to map a CP_INTC event directly to a core hardware interrupt, it must
 * first be mapped to a megamodule event with cpintc_map(). Then the megamodule
 * event can be mapped to a core hardware interrupt with irq_map(). To unmap,
 * first unmap the megamodule event, then the CP_INTC event.
 */
void cpintc_map(unsigned int irq_src, unsigned int irq_dst)
{
	struct combiner_handler_info *info;
	struct irq_desc     *desc;
	struct c6x_irq_chip *chip, *mmchip;
	unsigned long        flags;
	unsigned int         irq;
	int                  src;
	int                  output = -1;
	int                  i;

	/* Scan host IRQ to retrived output interrupt number */
	for (i = 0; i < IRQ_CPINTC0_MAPLEN; i++) {
		irq = cpintc_host_irq_map[i];
		if (irq == irq_dst)
			output = i;
	}

	/* Check that dst IRQ is a CPINTC IRQ*/
	if (output < 0)
		return;
	
	/* Only map CIC event sources */
	if (!((irq_src >= IRQ_CPINTC0_START)
	      && (irq_src < IRQ_CPINTC0_START + NR_CPINTC0_IRQS)))
		return;
	
	src = irq_src - IRQ_CPINTC0_START;

	spin_lock_irqsave(&map_lock, flags);

	/* Record the mapping */
	cpintc_evt_to_output[src] = output;

	/* Update the associated irq_chip */
	mmchip = &megamod_chips[output / 32];
	info   = &megamod_handler_info[output / 32];

	info->irqmap[output & 31] = irq_src;

	desc  = irq_to_desc(irq_src);
	chip  = &cpintc_mapped_chips[output];
	*chip = *(struct c6x_irq_chip *)desc->chip;

	chip->chip.name = mmchip->chip.name;
	chip->minfo     = mmchip->minfo;

	desc->chip = (struct irq_chip *)chip;

	/* Do the mapping within the CP_INTC hw */
	__cpintc_map_irq(__get_cpintc_id(), src, output);

	spin_unlock_irqrestore(&map_lock, flags);
}
EXPORT_SYMBOL(cpintc_map);

/*
 * Retrieve the corresponding mapped INTC IRQ for a given CP_INTC IRQ 
 * (combined or not)
 * 
 * CP_INTC IRQs mapped directly to megamodule use chip data from the megamodule
 * combiner since the megamodule combiner will controlling masking. All other
 * CP_INTC IRQs use CP_INTC combiner chip data.
 */
int irq_soc_mapped_irq(uint16_t irq)
{
#if 0
	uint8_t output;

	/* Not a CP_INTC irq */
	if (irq < IRQ_CPINTC0_START || irq >= (IRQ_CPINTC0_START + NR_CPINTC_IRQS))
		return irq;

	output = cpintc_evt_to_output[irq - IRQ_CPINTC0_START];

	DPRINTK("retrieve irq mapping for irq = %d, output = %d\n", irq, output);

	return (cpintc_host_irq_map[output]);
#else
	/* No combiner, use mapping n->1 instead */
	return irq;
#endif
}

EXPORT_SYMBOL(irq_soc_mapped_irq);

/*
 * Retrieve the corresponding CP_INTC event for a given INTC irq number
 */
int irq_soc_get(uint16_t irq_src, uint16_t *soc_irq)
{
	/* No combiner, use mapping n->1 instead */
	*soc_irq = irq_src;

	/* Always considered like combined */
	return 1;
}
EXPORT_SYMBOL(irq_soc_get);

static void cpintc_mask_combined(unsigned int irq)
{
	struct c6x_irq_chip *chip = c6x_irq_to_chip(irq);

	*chip->minfo->evtclr = (1 << (irq - chip->minfo->irq_base));
	DPRINTK("masking irq %d\n", irq);
}

static void cpintc_unmask_combined(unsigned int irq)
{
	struct c6x_irq_chip *chip = c6x_irq_to_chip(irq);

	*chip->minfo->evtset = (1 << (irq - chip->minfo->irq_base));
	DPRINTK("unmasking irq %d\n", irq);
}

#define CPINTC_CHIP(i)	\
	__CHIP("cp-combiner-", i, cpintc_mask_combined, cpintc_unmask_combined)

static struct c6x_irq_chip cpintc_chips[] = {
	CPINTC_CHIP(0),
	CPINTC_CHIP(1),
	CPINTC_CHIP(2),
	CPINTC_CHIP(3),
	CPINTC_CHIP(4),
	CPINTC_CHIP(5),
	CPINTC_CHIP(6),
	CPINTC_CHIP(7),
#if NR_CPINTC_COMBINERS > 8
	CPINTC_CHIP(8),
	CPINTC_CHIP(9),
	CPINTC_CHIP(10),
	CPINTC_CHIP(11),
	CPINTC_CHIP(12),
	CPINTC_CHIP(13),
	CPINTC_CHIP(14),
	CPINTC_CHIP(15),
#endif
};

/*
 * Initialise the CP_INTC controller data
 */
int irq_soc_init(void)
{
	int i, j;
	struct combiner_mask_info    *minfo;
	struct combiner_handler_info *hinfo;

	minfo = &cpintc_mask_info[0];
	hinfo = &cpintc_handler_info[0];
	for (i = 0; i < NR_CPINTC0_COMBINERS; i++) {
		minfo->irq_base = IRQ_CPINTC0_START + (i * 32);
		minfo->evtmask  = 0; 
		minfo->evtset	= &CPINTC_ENABLE(__get_cpintc_id())[i];
		minfo->evtclr	= &CPINTC_ENABLECLR(__get_cpintc_id())[i];
		hinfo->mevtflag = &CPINTC_ENASTATUS(__get_cpintc_id())[i]; /* status*/
		hinfo->evtclr   = &CPINTC_ENASTATUS(__get_cpintc_id())[i]; /* write 1 clean irq */
		for (j = 0; j < 32; j++)
			/* Define the default mapping */
			hinfo->irqmap[j] = minfo->irq_base + j;
		minfo++;
		hinfo++;
	}

	memset(&cpintc_evt_to_output[0], 0, sizeof(cpintc_evt_to_output));

	return 0;
}
EXPORT_SYMBOL(irq_soc_init);

/*
 * Setup the CP_INTC controller
 */
int irq_soc_setup(struct c6x_irq_chip          *parent_chips,
		  struct combiner_handler_info *parent_handler_info,
		  void (*handler)(unsigned int irq, struct irq_desc *desc),
		  struct irqaction             *combiner_actions)
{
	int i;
	struct irq_desc *desc;
	struct combiner_handler_info *hinfo;

	/* Disable all host interrupts */
	*CPINTC_GLOBALHINTEN(__get_cpintc_id()) = 0;

	/* Configure CP_INTC with no nesting support */
	*CPINTC_CTRL(__get_cpintc_id()) = CPINTC_NO_NESTING;

	/* Default settings for CP-INTC host interrupt */
	for (i = 0; i < IRQ_CPINTC0_MAPLEN; i++) {
		unsigned int irq = cpintc_host_irq_map[i];

		desc = irq_to_desc(irq);

		desc->status |= (IRQ_NOREQUEST | IRQ_NOPROBE);
		set_irq_chip(irq, &dummy_irq_chip);
		set_irq_handler(irq, handle_bad_irq);
	}

	/* 
	 * Configure CP_INTC combined IRQs to INTC INTCn IRQs.
	 */
	for (i = 0; i < NR_CPINTC0_COMBINERS; i++) {
		int irq, parent_irq;
		int j;
		unsigned int chan = __idx_to_chan(i);
		struct c6x_irq_chip *chip;

		CPINTC_ENABLECLR(__get_cpintc_id())[i] = ~0;	/* mask all events */
		CPINTC_ENASTATUS(__get_cpintc_id())[i] = ~0;	/* clear all events */

		/*
		 * CP_INTC has no default combiner, so we need to map all incoming
		 * events to host interrupt:
		 * evt 0 - 31    -> channel 0
		 * evt 32 - 63   -> channel 1
		 * ...
		 * evt 224 - 155 -> channel 7
		 */
		for (j = 0; j < 32; j++) {
			unsigned int evt = j + (i << 5);

			/* Check that evt IRQ is a real CP_INTC IRQ */
			if (evt >= NR_CPINTC0_IRQS)
			    continue;
	
			/* Map the event (system interrupt) to the combined channel */
			__cpintc_map_irq(__get_cpintc_id(), evt, chan);

			/* Record the mapping */
			cpintc_evt_to_output[evt] = chan;
			
			/* Set the individiual CP_INTC IRQs to use combiner */
			set_irq_chip(evt + IRQ_CPINTC0_START, (struct irq_chip *)&cpintc_chips[i]);
			set_irq_handler(evt + IRQ_CPINTC0_START, handle_level_irq);
		}

		/* Retrieve the mapping channel -> host irq */
		irq = __cpintc_get_host_irq(__get_cpintc_id(), chan);
		if (irq == -1) {
			/* Map the corresponding channel to the host interrupt */
			__cpintc_map_channel(__get_cpintc_id(), chan, i);
			irq = i;
		}

		/* Enable the corresponding output host interrupt */
		*CPINTC_HINTIDXSET(__get_cpintc_id()) = irq;

		cpintc_chips[i].minfo = &cpintc_mask_info[__host_irq_to_idx(irq)];

		/* Mapping of irq in the parent int controller (INTC) */
		parent_irq = cpintc_host_irq_map[__host_irq_to_idx(irq)];
		DPRINTK("irq = %d, channel = %d, combiner = %d, parent_irq = %d\n",
			irq, chan , i, parent_irq);

		/* Chip info for parent int controller we are wired through (INTC) */
		chip  = &parent_chips[parent_irq / 32];
		hinfo = &parent_handler_info[parent_irq / 32];

		set_irq_chip(parent_irq, (struct irq_chip *)chip);
		DPRINTK("parent_irq = %d, chip = 0x%x\n",
			parent_irq, chip);

		set_irq_handler(parent_irq, handler);
		set_irq_data(parent_irq, &cpintc_handler_info[i]);

		desc         = irq_to_desc(parent_irq);
		desc->action = &combiner_actions[NR_MEGAMOD_COMBINERS + i];
		chip->chip.startup(parent_irq);
	}

	/* Enable all host interrupts */
	*CPINTC_GLOBALHINTEN(__get_cpintc_id()) = 1;

	megamod_chips        = parent_chips;
	megamod_handler_info = parent_handler_info;

	spin_lock_init(&map_lock);

	return 0;
}
EXPORT_SYMBOL(irq_soc_setup);
