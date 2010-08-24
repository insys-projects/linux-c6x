/*
 *  linux/arch/c6x/drivers/tsc-c64xplus.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Contributed by: Mark Salter <msalter@redhat.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/percpu.h>
#include <linux/smp.h>


#ifdef CONFIG_GENERIC_TIME
static cycle_t tsc_read(struct clocksource *cs)
{
	return get_cycles();
}

static struct clocksource clocksource_tsc = {
	.name		= "TSC64",
	.rating		= 300,
	.read		= tsc_read,
	.mask		= CLOCKSOURCE_MASK(64),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

int __init c6x_arch_init_clocksource(void)
{
	struct clocksource *cs = &clocksource_tsc;
	u64 temp;
	u32 shift;

	/* Find a shift value */
	for (shift = 32; shift > 0; shift--) {
		temp = (u64) NSEC_PER_SEC << shift;
		do_div(temp, (CONFIG_TMS320C6X_MHZ*1000000));
		if ((temp >> 32) == 0)
			break;
	}
	cs->shift = shift;
	cs->mult = (u32) temp;

	/* write anything into TSCL to enable counting */
	TSCL = 0;

	clocksource_register(cs);

	return 0;
}
#endif /* CONFIG_GENERIC_TIME */

