/*
 *  linux/arch/c6x/drivers/timer-tci648x.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Contributed by: Mark Salter (msalter@redhat.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/percpu.h>
#include <linux/smp.h>

#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/timer.h>
#include <asm/system.h>
#include <asm/bug.h>
#include <asm/dscr.h>

#include <mach/board.h>

#ifdef CONFIG_GENERIC_CLOCKEVENTS

unsigned int timer_clock_divisor;

static int next_event(unsigned long delta,
		      struct clock_event_device *evt)
{
	u32 timer_CNTLO = TIMER_CNTLO_REG(LINUX_TIMER_SRC);
	u32 timer_PRDLO = TIMER_PRDLO_REG(LINUX_TIMER_SRC);
	u32 timer_TCR	= TIMER_TCR_REG(LINUX_TIMER_SRC);
	u32 timer_TGCR	= TIMER_TGCR_REG(LINUX_TIMER_SRC);

	TIMER_REG(timer_TCR)  &= ~TIMER_B_TCR_ENAMODELO_MASK;
	TIMER_REG(timer_PRDLO) = delta - 1;
	TIMER_REG(timer_CNTLO) = 0;
	TIMER_REG(timer_TCR) |= TIMER_B_TCR_ENAMODELO_ONCE;

	return 0;
}

static void set_clock_mode(enum clock_event_mode mode,
			   struct clock_event_device *evt)
{
#if 0
	u32 timer_TCR  = TIMER_TCR_REG(LINUX_TIMER_SRC);
	u32 timer_TGCR = TIMER_TGCR_REG(LINUX_TIMER_SRC);

	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* disable timer */
		TIMER_REG(timer_TCR)  &= ~TIMER_B_TCR_ENAMODELO_MASK;
		break;
	case CLOCK_EVT_MODE_RESUME:
		/* disable timer */
		TIMER_REG(timer_TCR)  &= ~TIMER_B_TCR_ENAMODELO_MASK;
		break;
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	}
#endif
}

static void event_handler(struct clock_event_device *dev)
{
}

static struct clock_event_device t64_clockevent_device;

static irqreturn_t timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *cd = &t64_clockevent_device;
	volatile unsigned long nl, nh;

	cd->event_handler(cd);

	return IRQ_HANDLED;
}

int __init c6x_arch_init_clockevents(void)
{
	struct clock_event_device *cd = &t64_clockevent_device;
	u64 temp;
	u32 shift, timer_TCR, timer_TGCR;
	u32 timer_PRDLO, timer_CNTLO, timer_EMUMGTCLKSPD;

	timer_TCR	   = TIMER_TCR_REG(LINUX_TIMER_SRC);
	timer_TGCR	   = TIMER_TGCR_REG(LINUX_TIMER_SRC);
	timer_CNTLO	   = TIMER_CNTLO_REG(LINUX_TIMER_SRC);
	timer_PRDLO	   = TIMER_PRDLO_REG(LINUX_TIMER_SRC);
	timer_EMUMGTCLKSPD = TIMER_EMUMGTCLKSPD_REG(LINUX_TIMER_SRC);

	/* disable timer, reset count */
	TIMER_REG(timer_TCR)  &= ~TIMER_B_TCR_ENAMODELO_MASK;
	TIMER_REG(timer_PRDLO) = 0;

	/* use internal clock and 1 cycle pulse width */
	TIMER_REG(timer_TCR)  &= ~(TIMER_B_TCR_CLKSRCLO | TIMER_B_TCR_PWIDLO_MASK);

	/* dual 32-bit unchained mode */
	TIMER_REG(timer_TGCR) &= ~TIMER_B_TGCR_TIMMODE_MASK;
	TIMER_REG(timer_TGCR) |= (TIMER_B_TGCR_TIMLORS | TIMER_B_TGCR_TIMMODE_UD32);

	timer_clock_divisor = (TIMER_REG(timer_EMUMGTCLKSPD) & (0xf << 16)) >> 16;

	cd->irq		= LINUX_TIMER_IRQ;
	cd->name	= "TIMER64_EVT32_TIMER";
	cd->features	= CLOCK_EVT_FEAT_ONESHOT;

	/* Calculate the min / max delta */
	/* Find a shift value */
	for (shift = 32; shift > 0; shift--) {
		temp = (u64) ((CONFIG_TMS320C6X_MHZ*1000000)/timer_clock_divisor) << shift;
		do_div(temp, NSEC_PER_SEC);
		if ((temp >> 32) == 0)
			break;
	}
	cd->shift = shift;
	cd->mult = (u32) temp;

	cd->max_delta_ns	= clockevent_delta2ns(0x7fffffff, cd);
	cd->min_delta_ns	= clockevent_delta2ns(250, cd);

	cd->rating		= 200;
	cd->set_mode		= set_clock_mode;
	cd->event_handler	= event_handler;
	cd->set_next_event	= next_event;
	cd->cpumask		= cpumask_of(smp_processor_id());

	clockevents_register_device(cd);

	/* Set handler */
	request_irq(cd->irq, timer_interrupt, IRQF_DISABLED | IRQF_TIMER,
		    "timer", NULL);

	return 0;
}

#endif /* CONFIG_GENERIC_CLOCKEVENTS */

