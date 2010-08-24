/*
 *  linux/arch/c6x/platforms/include/mach/irq.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Mark Salter <msalter@redhat.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __MACH_C6X_IRQ_H
#define __MACH_C6X_IRQ_H

#if defined(CONFIG_SOC_TMS320C6455)
#include <mach/irq-c645x.h>
#elif defined(CONFIG_SOC_TMS320C6472)
#include <mach/irq-c6472.h>
#elif defined(CONFIG_SOC_TMS320C6474)
#include <mach/irq-c6474.h>
#else
#error "No machine IRQ definitions"
#endif

#if defined(CONFIG_SOC_TMS320C6472) || defined(CONFIG_SOC_TMS320C6474)
static inline void assert_event(unsigned int event) 
{
	volatile unsigned int * reg = (volatile unsigned int *) IRQ_EVTASRT_REG;
	*reg = event;
}
#endif

#endif /* __MACH_C6X_IRQ_H */
