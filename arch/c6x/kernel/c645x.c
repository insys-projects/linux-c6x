/*
 *  linux/arch/c6x/kernel/c645x.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2006, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/kernel_stat.h>
#include <linux/types.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/timer.h>
#include <asm/system.h>
#include <asm/bug.h>
#include <asm/dscr.h>

/*
 * Resources present on the C645x chip
 */
struct resource c645x_chipreg_res = { "C645x peripherals", 0x01800000, 0x02cffffff };

static unsigned int clock_diviser; 

/*
 * hook for getting the time offset.  Note that it is
 * always called with interrupts disabled.
 */
unsigned long c645x_gettimeoffset(void)
{
	unsigned int timer_CNTLO = TIMER_CNTLO_REG(TIMER_1);
	unsigned int count       = TIMER_REG(timer_CNTLO);

	return ticks2usecs(CONFIG_TMS320C6X_MHZ, clock_diviser, count);
}

/*
 * Setup the general timer: use the C645x timer 1 in 32 bits mode
 */
void c645x_setup_timer(void) 
{
	unsigned long val;

	/* Enable timers (in regs PERLOCK & PERCFG0) */
	val = dscr_get_reg(DSCR_PERCFG0);
	dscr_set_device(val | DSCR_B_PERCFG0_TIMER0 | DSCR_B_PERCFG0_TIMER1, DSCR_PERCFG0);

	/* Wait for enabling (reg PERSTAT0) */
	val = 0;
	while (val != 0x9) {
	    val = dscr_get_reg(DSCR_PERSTAT0);
	    val = ((val & 0x3E00) >> 9);
	}

#if defined(CONFIG_GENERIC_TIME)
	init_tsc_clocksource();
#endif
#if defined(CONFIG_GENERIC_CLOCKEVENTS)
	init_timer64_clockevents();
#endif
}

void proc_setup_config(void)
{
 	/* Initialize C64x+ IRQs */          	
#ifndef CONFIG_NK
	clear_all_irq();                /* acknowledge all pending irqs */
#else
	irq_IER = 0;
#endif
}

#if defined(CONFIG_USE_RT_TIMER)

#define USECS_PER_JIFFY	        (1000000/HZ)

static unsigned long long previous_usec = 0;
static unsigned long previous_ilatency  = 0;

static unsigned long long _getrttimeoffset(void)
{
	unsigned int counter_low;
	unsigned long long counter_high;

	__dint();
	counter_low  = TSCL;
	counter_high = TSCH;
	__rint();

	return ((counter_high << 32) | counter_low) / CONFIG_TMS320C6X_MHZ;
}

void adjust_time(void) {
	long diff;
	unsigned long long usec = _getrttimeoffset();

	diff = usec - previous_usec;

	/* It is not worth checking for roll-over (roll-over after 585 years) */
	if (diff < 0)
		BUG();

	diff += previous_ilatency;

	/* do we lost some jiffies ? */
	if (diff > USECS_PER_JIFFY) {
     		diff -= USECS_PER_JIFFY;
		while (diff >= USECS_PER_JIFFY) {
			jiffies++;

			if (mach_leds_timer != NULL)
				mach_leds_timer();

			diff -= USECS_PER_JIFFY;
		}
	}

	previous_usec = usec;

	/* get current timer interrupt latency */
	previous_ilatency = c645x_gettimeoffset();
}
#endif /* CONFIG_USE_RT_TIMER */

