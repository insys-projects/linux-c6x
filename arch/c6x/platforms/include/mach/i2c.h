/*
 *  linux/arch/c6x/platforms/include/mach/i2c.h
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
#ifndef __MACH_C6X_I2C_H
#define __MACH_C6X_I2C_H

#include <asm/hardware.h>
#include <asm/irq.h>

#ifndef __ASSEMBLY__
/* 
 * This definition must match the davinci i2c driver structure.
 * All frequencies are expressed in kHz
 */
struct davinci_i2c_platform_data {
	unsigned int	bus_freq;	/* standard bus frequency (kHz) */
	unsigned int	bus_delay;	/* post-transaction delay (usec) */
	unsigned int    sda_pin;        /* GPIO pin ID to use for SDA (UNUSED) */
	unsigned int    scl_pin;        /* GPIO pin ID to use for SCL (UNUSED) */
};
#endif

/*
 * I2C registers base
 */
#ifdef CONFIG_TMS320C66X
#define ARCH_I2C_BASE    0x02530000
#else
#define ARCH_I2C_BASE    0x02B04000
#endif

#endif  /* __MACH_C6X_I2C_H */
