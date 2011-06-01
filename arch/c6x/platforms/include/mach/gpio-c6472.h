/*
 *  arch/c6x/platforms/include/mach/gpio-c6472.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __MACH_GPIO_C6472_H
#define __MACH_GPIO_C6472_H

#include <mach/irq.h>

#define SOC_GPIO_BASE 0x02B00000

static inline unsigned __mach_gpio_irq_to_mask(unsigned irq)
{
	if (irq == IRQ_GPIOINT)
		return 1 << get_coreid();
	
	return 1 << (CORE_NUM + irq - IRQ_GPIO_START - 1);
}

static inline unsigned __mach_gpio_to_irq(unsigned offset)
{
	if (offset == get_coreid())
		return IRQ_GPIOINT;
	else if (offset < CORE_NUM)
		return -ENODEV;
	offset -= (CORE_NUM - 1);

	return IRQ_GPIO_START + offset;
}

/* 
 * Define the list of IRQ for GPIO interrupts 
 */
#define MACH_GPIO_IRQ_LIST_DEF()					\
	{ IRQ_GPIOINT, IRQ_GPIO6, IRQ_GPIO7, IRQ_GPIO8, IRQ_GPIO9, IRQ_GPIO10, \
	  IRQ_GPIO11, IRQ_GPIO12, IRQ_GPIO13, IRQ_GPIO14, IRQ_GPIO15 }

#endif /* __MACH_GPIO_C6472_H */
