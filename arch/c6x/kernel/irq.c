/*
 *  linux/arch/c6x/kernel/irq.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2006, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  C6x general interrupt handling code.
 */
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kernel_stat.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/hardirq.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/traps.h>
#include <asm/page.h>
#include <asm/machdep.h>
#include <asm/hardirq.h>
#include <asm/hardware.h>

#include <asm/percpu.h>

DECLARE_PER_CPU(struct kernel_stat, kstat);

/*
 * Mach dep functions
 */ 
/* table for system interrupt handlers */
static unsigned int irq_mux[SYS_IRQS];
#ifdef IRQ_USE_CIC
static unsigned int cic_mux[CIC_IRQS];
#endif

static void c64x_unmask_irq(unsigned int irq)
{
	enable_irq_mask(1 << irq);
}

static void c64x_mask_irq(unsigned int irq)
{
	disable_irq_mask(1 << irq);
}

static struct irq_chip c64x_irq_chip = {
	.name		= "C64x+",
	.mask		= c64x_mask_irq,
	.unmask		= c64x_unmask_irq,
};

/* The number of spurious interrupts */
volatile unsigned int num_spurious;

#ifdef CONFIG_TMS320C64X
/*
 * Map a C64x interrupt source on a CPU interrupt using MUX registers
 */
void irq_map(unsigned int irq_src, unsigned int cpu_irq)
{
	unsigned int offset;
	volatile unsigned int* reg;
	unsigned int flags;

	if (cpu_irq < INT4 || cpu_irq > INT15)
		return;
		
	if ((cpu_irq >= INT4) && (cpu_irq <= INT9)) {
		offset = ((cpu_irq - INT4) * 5);
		reg = (unsigned int *) IRQ_MUXL_REG;
	} else {
		offset = ((cpu_irq - INT10) * 5);
		reg = (unsigned int *) IRQ_MUXH_REG;
	}
	if (offset > 14)
		offset++;
	
	save_global_flags(flags);
	global_cli();

	*reg &= ~(0x1f << offset);
	*reg |= ((irq_src & 0x1f) << offset);

	restore_global_flags(flags);

	irq_mux[cpu_irq] = irq_src;
}
EXPORT_SYMBOL(irq_map);

/*
 * Change polarity of a given interrupt source
 */
void irq_set_polarity(unsigned int irq_src, unsigned int polarity)
{
	volatile unsigned int xip;
	unsigned int flags;

	if (irq_src < IRQ_EXTINT4 || irq_src > IRQ_EXTINT7)
		return;

	save_global_flags(flags);
	global_cli();

	xip = *((volatile unsigned int *) IRQ_EXTPOL_REG);
	xip |= (polarity & 1) << (irq_src - IRQ_EXTINT4);
	*((volatile unsigned int *) IRQ_EXTPOL_REG) = xip;

	restore_global_flags(flags);	
}
#endif /* CONFIG_TMS320C64X */


#ifdef CONFIG_TMS320C64XPLUS
/*
 * Map a C64x+ interrupt source on a CPU interrupt using MUX registers
 */
void irq_map(unsigned int irq_src, unsigned int cpu_irq)
{
	unsigned int offset = 0;
	volatile unsigned int* reg;
	
	if (cpu_irq < INT4 || cpu_irq > INT15)
		return;
		
	if ((cpu_irq >= INT4) && (cpu_irq <= INT7)) {
		offset = ((cpu_irq - INT4) << 3);
		reg = (unsigned int *) IRQ_INTMUX1_REG;
	}
 
	if ((cpu_irq >= INT8) && (cpu_irq <= INT11)) {
		offset = ((cpu_irq - INT8) << 3);
		reg = (unsigned int *) IRQ_INTMUX2_REG;
	}

	if ((cpu_irq >= INT12) && (cpu_irq <= INT15)) {
		offset = ((cpu_irq - INT12) << 3);
		reg = (unsigned int *) IRQ_INTMUX3_REG;
	}

	__dint();
	*reg &= ~(0x7f << offset);
	*reg |= ((irq_src & 0x7f) << offset);
	__rint();

	irq_mux[cpu_irq] = irq_src;
}
EXPORT_SYMBOL(irq_map);

#ifdef IRQ_USE_CIC
/*
 * Acknowledge a given CIC input event
 */
void cic_raw_ack(unsigned int cic_src, unsigned int cic_id)
{
	unsigned int src;
	unsigned int offset1;
	unsigned int offset2;
	unsigned int reg_base = CIC0_REG_BASE + ((CIC1_REG_BASE - CIC0_REG_BASE) * cic_id);

	if (cic_src < 32) {
	        offset1 = CIC_EVTFLAG0_REG;
		offset2 = CIC_EVTCLR0_REG;
		src = cic_src;
	} else {
	        offset1 = CIC_EVTFLAG1_REG;
		offset2 = CIC_EVTCLR1_REG;
		src = cic_src - 32;
	}

	*((volatile unsigned int *)(reg_base + offset2)) = *((volatile unsigned int *)(reg_base + offset1))
	    & (1 << src);
}

void cic_ack(unsigned int cic_src)
{
	cic_raw_ack(cic_src, get_coreid());
}

/*
 * Map one of the 64 CIC input events to one of the 16 CIC output events for a given CIC
 */
void cic_raw_map(unsigned int cic_src, unsigned int cic_evt, unsigned int cic_id)
{
	unsigned int offset;
	unsigned int reg_base;

        if (cic_evt > CIC15)
	        return;

	/* ACK the CIC source */
	cic_raw_ack(cic_src, cic_id);

	cic_mux[cic_evt] = cic_src;

	reg_base = CIC0_REG_BASE + ((CIC1_REG_BASE - CIC0_REG_BASE) * cic_id);
	offset   = CIC_EVTMUX0_REG + ((cic_evt >> 2) << 2);

	__dint();
	*((volatile unsigned int *)(reg_base + offset))
		&= ~(0x3f << ((cic_evt & 0x3) << 3));
	*((volatile unsigned int *)(reg_base + offset))
		|= (cic_src & 0x3f) << ((cic_evt & 0x3) << 3);
	__rint();
}

/*
 * Map one of the 64 CIC input events to one of the 16 CIC IRQ events of the C64x+ core 
 */
void cic_map(unsigned int cic_src, unsigned int cic_irq)
{
	cic_raw_map(cic_src, cic_irq, get_coreid());
}

EXPORT_SYMBOL(cic_raw_ack);
EXPORT_SYMBOL(cic_ack);
EXPORT_SYMBOL(cic_raw_map);
EXPORT_SYMBOL(cic_map);
#endif /* IRQ_USE_CIC */
#endif /* CONFIG_TMS320C64XPLUS */

static atomic_t irq_err_count;
void ack_bad_irq(int irq)
{
	atomic_inc(&irq_err_count);
	printk(KERN_ERR "IRQ: spurious interrupt %d\n", irq);
}

static struct irq_desc bad_irq_desc = {
	.handle_irq = handle_bad_irq,
	.lock = __RAW_SPIN_LOCK_UNLOCKED(bad_irq_desc.lock),
};

asmlinkage void c6x_do_IRQ(unsigned int irq, struct pt_regs *regs)
{
	struct pt_regs *old_regs = set_irq_regs(regs);

	irq_enter();

/*	check_stack_overflow(irq); */

	if (irq >= NR_IRQS)
		handle_bad_irq(irq, &bad_irq_desc);
	else
		generic_handle_irq(irq);

#ifdef IRQ_USE_CIC
	/* Eventually ACK the CIC interrupt */
	if ((irq_mux[irq] >= IRQ_CICEVT_START) && 
	    (irq_mux[irq] <= IRQ_CICEVT_END))
	        cic_ack(cic_mux[irq_mux[irq] - IRQ_CICEVT0]);
#endif

	irq_exit();

	set_irq_regs(old_regs);
}

void __init init_IRQ(void)
{
	int i;

	for (i = 0; i < SYS_IRQS; i++) {
		irq_mux[i] = -1;
		set_irq_chip(i, &c64x_irq_chip);
		set_irq_handler(i, handle_level_irq);
	}

	if (mach_init_IRQ != NULL)
		mach_init_IRQ();
}

/* This is only necessary to squelch some warning about
 * IRQs being enabled early. radix_tree_init() leaves
 * interrupts enabled during early boot when they should
 * be disabled. None of the individual bits in IER are
 * enabled so we don't actually get interrupts, just the
 * warning from init/main.c boot code.
 */
int __init __weak arch_early_irq_init(void)
{
	/* clear interrupt flags */
	set_creg(ICR, 0xfff0);
	local_irq_disable();
	return 0;
}


int show_interrupts(struct seq_file *p, void *v)
{
	int i = *(loff_t *) v, j, cpu;
	struct irqaction * action;
	unsigned long flags;

	if (i == 0) {
		seq_printf(p, "           ");
		for (j = 0; j < NR_CPUS; j++)
			if (cpu_online(j))
				seq_printf(p, "CPU%d       ", j);
		seq_putc(p, '\n');
	} else if (i < NR_IRQS) {
		raw_spin_lock_irqsave(&irq_desc[i].lock, flags);
		action = irq_desc[i].action;
		if (action) {
			seq_printf(p, "%3d: ", i);
			for_each_present_cpu(cpu)
				seq_printf(p, "%10u ", kstat_irqs_cpu(i, cpu));
			seq_printf(p, " %10s", irq_desc[i].chip->name ? : "-");
			seq_printf(p, "  %s", action->name);
			for (action = action->next; action; action = action->next)
				seq_printf(p, ", %s", action->name);

			seq_putc(p, '\n');
		}
		raw_spin_unlock_irqrestore(&irq_desc[i].lock, flags);
	}
	return 0;
}

