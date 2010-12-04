/*
 *  linux/arch/c6x/kernel/time.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/interrupt.h>

#include <asm/machdep.h>
#include <asm/segment.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/timer.h>

#include <linux/timex.h>
#include <linux/profile.h>

extern void (* mach_setup_timer)(void);

static u32 sched_clock_multiplier;
#define SCHED_CLOCK_SHIFT 16

/*
 * scheduler clock - returns current time in nanosec units.
 */
u64 sched_clock(void)
{
	u64 tsc;

	/* read the TSC value
	 */
	tsc = get_cycles();

	return ((tsc * sched_clock_multiplier) >> SCHED_CLOCK_SHIFT);
}

extern void arch_gettod(int *year, int *mon, int *day, int *hour, int *min, int *sec);

void time_init(void)
{
	int year, mon, day, hour, min, sec;
	
	arch_gettod(&year, &mon, &day, &hour, &min, &sec);

	if ((year += 1900) < 1970)
		year += 100;

	xtime.tv_sec  = mktime(year, mon, day, hour, min, sec);
	xtime.tv_nsec = 0;

	wall_to_monotonic.tv_sec = -xtime.tv_sec;

	sched_clock_multiplier = ((u64)NSEC_PER_SEC << SCHED_CLOCK_SHIFT) / c6x_core_freq;

	if (mach_setup_timer != NULL)
		mach_setup_timer();
	else {
#if defined(CONFIG_GENERIC_TIME)
		c6x_arch_init_clocksource();
#endif
#if defined(CONFIG_GENERIC_CLOCKEVENTS)
		c6x_arch_init_clockevents();
#endif
	}
}

