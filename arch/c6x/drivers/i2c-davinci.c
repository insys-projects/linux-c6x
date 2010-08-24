/*
 * Davinci I2C setup for C64X devices
 *
 * Copyright (C) 2010 Texas Instruments.
 * Contributed by: Mark Salter <msalter@redhat.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>

#include <mach/board.h>
#include <mach/i2c.h>

/*
 *  Not much to do here except register the data for the Davinci driver
 */

static struct resource i2c_resources[] = {
	{
		.start		= ARCH_DAVINCI_I2C_BASE,
		.end		= ARCH_DAVINCI_I2C_BASE + 0x40,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= IRQ_DAVINCI_I2C,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq	= ARCH_DAVINCI_I2C_FREQ,
	.bus_delay	= ARCH_DAVINCI_I2C_DELAY,
};

static struct platform_device c6x_i2c_device = {
	.name           = "i2c_davinci",
	.id             = 1,
	.num_resources	= ARRAY_SIZE(i2c_resources),
	.resource	= i2c_resources,
	.dev		= { .platform_data = &i2c_pdata },
};

static void __init c6x_init_i2c(void)
{
	platform_device_register(&c6x_i2c_device);
}

core_initcall(c6x_init_i2c);


