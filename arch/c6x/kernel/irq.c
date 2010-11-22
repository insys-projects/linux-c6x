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
#include <linux/io.h>

#include <asm/system.h>
#include <asm/irq.h>
#include <asm/traps.h>
#include <asm/page.h>
#include <asm/machdep.h>
#include <asm/hardirq.h>
#include <asm/hardware.h>

DECLARE_PER_CPU(struct kernel_stat, kstat);

#ifndef hw_to_kernel_irq
#define hw_to_kernel_irq(hw) (hw)
#endif

static void mask_core_irq(unsigned int irq)
{
	BUG_ON(irq >= NR_SYS_IRQS);
	disable_irq_mask(1 << irq);
}

static void unmask_core_irq(unsigned int irq)
{
	BUG_ON(irq >= NR_SYS_IRQS);
	enable_irq_mask(1 << irq);
}

static struct irq_chip core_irq_chip = {
	.name		= "C64x+",
	.mask		= mask_core_irq,
	.unmask		= unmask_core_irq,
};

void ack_bad_irq(int irq)
{
	printk(KERN_ERR "IRQ: spurious interrupt %d\n", irq);
}

asmlinkage void c6x_do_IRQ(unsigned int prio, struct pt_regs *regs)
{
	struct pt_regs *old_regs = set_irq_regs(regs);

	irq_enter();

	BUG_ON(prio >= NR_SYS_IRQS);

/*	check_stack_overflow(irq); */

	generic_handle_irq(hw_to_kernel_irq(prio));

	irq_exit();

	set_irq_regs(old_regs);
}

void __init init_IRQ(void)
{
	int i;
	struct irq_desc *desc;

	/* Mask all general IRQs */
	and_creg(IER, ~0xfff0);

	for (i = 0; i < NR_SYS_IRQS; i++) {
		desc = irq_to_desc(i);
		set_irq_chip_and_handler(i, &core_irq_chip, handle_level_irq);
	}

#ifdef CONFIG_PIC_C64XPLUS
	init_pic_c64xplus();
#endif
	if (mach_init_IRQ != NULL)
		mach_init_IRQ();

	/* Clear all general IRQ flags */
	set_creg(ICR, 0xfff0);
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
	}
	if (i < NR_IRQS) {
		raw_spin_lock_irqsave(&irq_desc[i].lock, flags);
		action = irq_desc[i].action;
		if (action) {
			seq_printf(p, "%3d: ", i);
			for_each_present_cpu(cpu)
				seq_printf(p, "%10u ", kstat_irqs_cpu(i, cpu));
			seq_printf(p, " %-14s", irq_desc[i].chip->name ? : "-");
			seq_printf(p, "  %s", action->name);
			for (action = action->next; action; action = action->next)
				seq_printf(p, ", %s", action->name);

			seq_putc(p, '\n');
		}
		raw_spin_unlock_irqrestore(&irq_desc[i].lock, flags);
	}
	return 0;
}

