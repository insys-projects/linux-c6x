/*
 *  linux/arch/c6x/drivers/cp-intc.c
 *
 *  Support for C66x Communication Port Interrupt Controller (CP_INTC)
 *
 *  Copyright (C) 2011, 2012 Texas Instruments Incorporated
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
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/hardware.h>
#include <asm/cp-intc.h>

#ifdef DEBUG
#define DPRINTK(fmt, args...) printk(KERN_CRIT "CP_INTC: [%s] " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...) 
#endif

#define MAX_CPINTC_PER_COMBINER 32

struct c6x_irq_chip {
	struct irq_chip	           chip;
	struct combiner_mask_info *minfo;
};

struct __cpintc_irq_info {
	volatile uint32_t *mevtflag;
	volatile uint32_t *evtclr;
	uint32_t           mask;
	uint16_t           irq;
	uint16_t           level;
};

struct cpintc_handler_info {
	uint16_t                 nb_irq;
	uint16_t                 combiner;
	struct __cpintc_irq_info irq_i[MAX_CPINTC_PER_COMBINER];
};

static struct combiner_mask_info     cpintc_mask_info[NR_CPINTC_COMBINERS];
static struct cpintc_handler_info    cpintc_handler_info[NR_CPINTC_COMBINERS];
static uint8_t                       cpintc_evt_to_output[NR_CPINTC_IRQS];
static struct c6x_irq_chip           cpintc_mapped_chips[NR_CPINTC0_OUTPUTS];

static char                          combiner_actions_name[NR_CPINTC_COMBINERS][20];

static struct c6x_irq_chip          *megamod_chips;
static struct combiner_handler_info *megamod_handler_info;
static void (*megamod_handler)(unsigned int irq, struct irq_desc *desc);
struct irqaction                    *megamod_actions;

#define __host_irq_to_idx(irq) IRQ_SOC_HOST_IRQ_TO_IDX(irq)
#define __idx_to_chan(i)       IRQ_SOC_IDX_TO_CHAN(i)
#define __irq_to_evt(irq)      (irq - IRQ_CPINTC0_START)
#define __evt_to_irq(evt)      (evt + IRQ_CPINTC0_START)

/*
 * There is one CP_INTC for 4 cores on C66x
 */
#if (CORE_NUM < 5)
#define __get_cpintc_id()      0
#else
#define __get_cpintc_id()      (get_coreid() >> 2)
#endif

/* Default chip operations for CP_INTC interrupts */
static void cpintc_mask(unsigned int irq);
static void cpintc_unmask(unsigned int irq);
static unsigned int cpintc_startup(unsigned int irq);
static void cpintc_disable(unsigned int irq);

/* Default CP_INTC chip mapped to CP_INTC events */
static struct irq_chip cpintc_default_chip = {
	.name     = "default_cpintc_chip",
	.startup  = cpintc_startup,
	.disable  = cpintc_disable,
	.mask     = cpintc_mask,
	.unmask   = cpintc_unmask,
};

/* Mapping from host int to INTC mapping */
static uint8_t cpintc_host_irq_map[] = { 
	IRQ_CPINTC0_COMBINER,
	IRQ_CPINTC0_COMBINER + 1,
	IRQ_CPINTC0_COMBINER + 2,
	IRQ_CPINTC0_COMBINER + 3,
	IRQ_CPINTC0_COMBINER + 4,
	IRQ_CPINTC0_COMBINER + 5,
	IRQ_CPINTC0_COMBINER + 6,
	IRQ_CPINTC0_COMBINER + 7,
	IRQ_CPINTC0_COMBINER + 8,
	IRQ_CPINTC0_COMBINER + 9,
	IRQ_CPINTC0_COMBINER + 10,
	IRQ_CPINTC0_COMBINER + 11,
};

/* Interrupts that are level instead of pulse */
u32 cpintc_level_irqs[] = IRQ_CPINTC_LEVEL_IRQS;

/* Lock protecting irq mappings */
static spinlock_t map_lock;

/* free CP_INTC combiner to allocate */
static unsigned int cpintc_free_combiner = 0;

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
	for (i = 0; i < NR_CPINTC_COMBINERS; i++) {
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

	BUG_ON(src >= NR_CPINTC0_IRQS);
	BUG_ON(channel >= NR_CPINTC0_CHANNELS);

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
	
	/* Only map CP_INTC event sources */
	if (!((irq_src >= IRQ_CPINTC0_START)
	      && (irq_src < IRQ_CPINTC0_START + NR_CPINTC0_IRQS)))
		return;
	
	src = __irq_to_evt(irq_src);

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
 */
int irq_soc_mapped_irq(uint16_t irq)
{
	return irq;
}

EXPORT_SYMBOL(irq_soc_mapped_irq);

/*
 * Retrieve the corresponding CP_INTC event for a given INTC irq number
 */
int irq_soc_get(uint16_t irq_src, uint16_t *soc_irq)
{
	*soc_irq = irq_src;

	/* Always considered like combined */
	return 1;
}
EXPORT_SYMBOL(irq_soc_get);

/*
 * Mask/Unmask a CP_INTC interrupt
 */
static void cpintc_mask(unsigned int irq)
{
	struct c6x_irq_chip *chip  = c6x_irq_to_chip(irq);
	unsigned int         evt   = __irq_to_evt(irq);
	
	*chip->minfo->evtclr = evt;
}

static void cpintc_unmask(unsigned int irq)
{
	struct c6x_irq_chip *chip  = c6x_irq_to_chip(irq);
	unsigned int         evt   = __irq_to_evt(irq);

	*chip->minfo->evtset = evt;
}

#define CPINTC_CHIP(i)	\
	__CHIP("cp-combiner-", i, cpintc_mask, cpintc_unmask)

static struct c6x_irq_chip cpintc_chips[] = {
	CPINTC_CHIP(0),
	CPINTC_CHIP(1),
	CPINTC_CHIP(2),
	CPINTC_CHIP(3),
	CPINTC_CHIP(4),
	CPINTC_CHIP(5),
	CPINTC_CHIP(6),
	CPINTC_CHIP(7),
	CPINTC_CHIP(8),
	CPINTC_CHIP(9),
	CPINTC_CHIP(10),
	CPINTC_CHIP(11),
	CPINTC_CHIP(12),
	CPINTC_CHIP(13),
	CPINTC_CHIP(14),
	CPINTC_CHIP(15),
};

/*
 * Initialise the CP_INTC controller data
 */
int irq_soc_init(void)
{
	int i;
	struct combiner_mask_info  *minfo;
	struct cpintc_handler_info *hinfo;

	minfo = &cpintc_mask_info[0];
	hinfo = &cpintc_handler_info[0];

	for (i = 0; i < NR_CPINTC_COMBINERS; i++) {
		minfo->irq_base = 0;
		minfo->evtmask  = 0; 
		minfo->evtset	= CPINTC_ENABLEIDXSET(__get_cpintc_id());
		minfo->evtclr	= CPINTC_ENABLEIDXCLR(__get_cpintc_id());
		hinfo->nb_irq   = 0;
		hinfo->combiner = 0xffff;
		minfo++;
		hinfo++;
	}

	memset(&cpintc_evt_to_output[0], 0, sizeof(cpintc_evt_to_output));

	return 0;
}
EXPORT_SYMBOL(irq_soc_init);

/*
 * Handler for combined CP_INTC interrupts
 */
static void handle_cpintc_combined_irq(unsigned int irq, struct irq_desc *desc)
{
	struct cpintc_handler_info *hinfo = get_irq_desc_data(desc);
	unsigned int i;

	raw_spin_lock(&desc->lock);

	if (unlikely(desc->status & IRQ_INPROGRESS))
		goto out_unlock;
	desc->status &= ~(IRQ_REPLAY | IRQ_WAITING);
	kstat_incr_irqs_this_cpu(irq, desc);

	desc->status |= IRQ_INPROGRESS;
	raw_spin_unlock(&desc->lock);

	/* Check all pending CP_INTC interrupts combined here, call handlers and ack */
	for (i = 0; i < hinfo->nb_irq; i++) {
		if (*hinfo->irq_i[i].mevtflag & hinfo->irq_i[i].mask) {
			unsigned int c_irq = hinfo->irq_i[i].irq; /* child IRQ to handle */
			void (*ack_handler)(unsigned int irq) = get_irq_chip_data(c_irq);
			
			/* Mask level interrupt */
			if (hinfo->irq_i[i].level)
				cpintc_mask(c_irq);

			/* Acknowledge interrupt */
			*hinfo->irq_i[i].evtclr = __irq_to_evt(c_irq);

			/* Call handler */
			generic_handle_irq(c_irq);

			/* Unmask level interrupt */
			if (hinfo->irq_i[i].level)
				cpintc_unmask(c_irq);

			/* We may have special acknowledge callbacks per irq (e.g. PCI INTx) */
			if (ack_handler)
				ack_handler(c_irq);
		}
	}

	raw_spin_lock(&desc->lock);
	desc->status &= ~IRQ_INPROGRESS;
out_unlock:
	raw_spin_unlock(&desc->lock);
}

/*
 * Disable a given CP_INTC interrupt: remove evt mapping to INTC combiner
 */
static void cpintc_disable(unsigned int irq)
{
	struct cpintc_handler_info *hinfo;
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned int c;

	hinfo = get_irq_desc_data(desc);
	if (!hinfo)
		goto exit;

	c = hinfo-> combiner;

	DPRINTK("Disable CP_INTC interrupt %d (attached to combiner %d)\n", irq, c);

	raw_spin_lock(&desc->lock);

	if (hinfo->nb_irq > 0) {
		unsigned i;
		for (i = 0; i < hinfo->nb_irq; i++) {
			if (hinfo->irq_i[i].irq == irq) {
				if (i < hinfo->nb_irq - 1)
					/* If needed, shift remaining irq struct */
					memcpy(&hinfo->irq_i[i],
					       &hinfo->irq_i[i+1],
					       sizeof(struct __cpintc_irq_info)
					       * (hinfo->nb_irq - i - 2));
				hinfo->nb_irq--;
			}
		}
	}
exit:
	raw_spin_unlock(&desc->lock);
}

/*
 * Check if a given interrupt is level or edge/pulse
 */
static unsigned int cpintc_irq_is_level(unsigned int irq)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cpintc_level_irqs); i++) {
		if (irq == cpintc_level_irqs[i]) {
			DPRINTK("Set irq %d as level signaled\n", irq);
			return 1;
		}
	}
	return 0;
}

/*
 * Startup a given CP_INTC interrupt: do dynamic mapping of the CP_INTC event
 * with the INTC combiner.
 */
static unsigned int cpintc_startup(unsigned int irq)
{
	struct c6x_irq_chip *chip;
	struct cpintc_handler_info *hinfo;
	struct irq_desc *desc = irq_to_desc(irq);
	unsigned int evt      = __irq_to_evt(irq);
	unsigned int c        = cpintc_free_combiner;
	unsigned int chan     = __idx_to_chan(c);
	unsigned int hirq;
	unsigned int parent_irq;

	DPRINTK("Starting up CP_INTC interrupt %d\n", irq);

	raw_spin_lock(&desc->lock);

	/* Map incoming CP_INTC events to a free host interrupt */
	cpintc_free_combiner += 1;
	if (cpintc_free_combiner >= NR_CPINTC0_COMBINERS) {
		cpintc_free_combiner = 0;
	}

	/* Check that evt IRQ is a real CP_INTC IRQ */
	if (evt >= NR_CPINTC0_IRQS) {
		DPRINTK("irq %d, bad CP_INTC interrupt (evt = %d)\n", irq, evt);
		raw_spin_unlock(&desc->lock);
		return 1;
	}
		
	/* Map the event (system interrupt) to the combined channel */
	__cpintc_map_irq(__get_cpintc_id(), evt, chan);

	/* Record the mapping */
	cpintc_evt_to_output[evt] = chan;
			
	/* Set the individiual CP_INTC IRQs to use combiner */
	set_irq_chip(irq, (struct irq_chip *)&cpintc_chips[c]);
	set_irq_handler(irq, handle_edge_irq);

	/* Retrieve the mapping channel -> host irq */
	hirq = __cpintc_get_host_irq(__get_cpintc_id(), chan);
	if (hirq == -1) {
		/* Map the corresponding channel to the host interrupt */
		__cpintc_map_channel(__get_cpintc_id(), chan, c);
		hirq = c;
	}

	/* Enable the corresponding output host interrupt */
	*CPINTC_HINTIDXSET(__get_cpintc_id()) = hirq;

	/* Mapping of irq in the parent int controller (INTC) */
	parent_irq = cpintc_host_irq_map[__host_irq_to_idx(hirq)];
	DPRINTK("host irq = %d, channel = %d, cpintc combiner = %d, parent_irq = %d\n",
		hirq, chan , c, parent_irq);

	/* Rename combiner used for CP_INTC */
	megamod_actions[NR_MEGAMOD_COMBINERS + c].name
		= &combiner_actions_name[c][0];
	sprintf((char *) megamod_actions[NR_MEGAMOD_COMBINERS + c].name,
		"cp-combiner-%d", c);

	cpintc_chips[c].minfo           = &cpintc_mask_info[__host_irq_to_idx(hirq)];
	cpintc_chips[c].minfo->irq_base = irq;

	hinfo = &cpintc_handler_info[c];

	BUG_ON(hinfo->nb_irq > MAX_CPINTC_PER_COMBINER);

	/* Set the CP_INTC registers to check status and clear interrupts and its info */
	hinfo->irq_i[hinfo->nb_irq].mevtflag = &CPINTC_ENASTATUS(__get_cpintc_id())[evt/32];
	hinfo->irq_i[hinfo->nb_irq].evtclr   = CPINTC_STATUSIDXCLR(__get_cpintc_id());
	hinfo->irq_i[hinfo->nb_irq].mask     = 1 << (evt & 31);
	hinfo->irq_i[hinfo->nb_irq].irq      = irq;
	hinfo->irq_i[hinfo->nb_irq].level    = cpintc_irq_is_level(irq);

       	hinfo->nb_irq++;
       	hinfo->combiner = c;

	desc->handler_data = hinfo;

	/* Chip info for parent int controller we are wired through (INTC) */
	chip = &megamod_chips[parent_irq / 32];
	
	set_irq_chip(parent_irq, (struct irq_chip *)chip);

	DPRINTK("parent_irq = %d, chip = 0x%x\n", parent_irq, chip);

	set_irq_handler(parent_irq, handle_cpintc_combined_irq);
	set_irq_data(parent_irq, hinfo);

	desc = irq_to_desc(parent_irq);
		
	/* Set parent (megamod) action to combiner but new handler info for CP_INTC */
	desc->action       = &megamod_actions[NR_MEGAMOD_COMBINERS + c];
	desc->handler_data = hinfo;

	chip->chip.startup(parent_irq);

	/* Unmask the interrupt */
	desc->chip->enable(irq);

	raw_spin_unlock(&desc->lock);

	return 0;
}

/*
 * Setup the CP_INTC controller
 */
int irq_soc_setup(struct c6x_irq_chip          *parent_chips,
		  struct combiner_handler_info *parent_handler_info,
		  void (*handler)(unsigned int irq, struct irq_desc *desc),
		  struct irqaction             *actions)
{
	int i;
	unsigned int evt;

	if (get_coreid() == get_master_coreid()) {
		/* Disable all host interrupts */
		*CPINTC_GLOBALHINTEN(__get_cpintc_id()) = 0;

		/* Configure CP_INTC with no nesting support */
		*CPINTC_CTRL(__get_cpintc_id()) = CPINTC_NO_NESTING;
	}

	/* Default settings for CP-INTC host interrupt */
	for (i = 0; i < IRQ_CPINTC0_MAPLEN; i++) {
		unsigned int irq      = cpintc_host_irq_map[i];
		struct irq_desc *desc = irq_to_desc(irq);

		desc->status |= (IRQ_NOREQUEST | IRQ_NOPROBE);
		set_irq_chip(irq, &dummy_irq_chip);
		set_irq_handler(irq, handle_bad_irq);
	}

	/* Initialize CP_INTC IRQs */
	for (evt = 0; evt < NR_CPINTC_IRQS; evt++) {
		unsigned int irq = __evt_to_irq(evt);

		if (get_coreid() == get_master_coreid()) {
			*CPINTC_ENABLEIDXCLR(__get_cpintc_id()) = evt; /* mask event */
			*CPINTC_STATUSIDXCLR(__get_cpintc_id()) = evt; /* clear event */
		}

		/* Set the individiual CP_INTC IRQs to do dynamical mapping */
		set_irq_chip(irq, &cpintc_default_chip);
	}

	if (get_coreid() == get_master_coreid()) {
		/* Enable all host interrupts */
		*CPINTC_GLOBALHINTEN(__get_cpintc_id()) = 1;
	}

	megamod_chips        = parent_chips;
	megamod_handler_info = parent_handler_info;
	megamod_actions      = actions;
	megamod_handler      = handler;

	spin_lock_init(&map_lock);

	return 0;
}
EXPORT_SYMBOL(irq_soc_setup);
