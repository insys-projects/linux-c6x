/*
 *  linux/include/asm-c6x/delay.h
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
#ifndef __ASM_C6X_DELAY_H
#define __ASM_C6X_DELAY_H

#include <linux/kernel.h>
#include <asm/timer.h>

extern asmlinkage void _c6x_delay(unsigned long);

static inline void __delay(unsigned long loop)
{
	_c6x_delay(loop / 3); /* Because a loop takes 6 cycles on C6x instead
				 of 2 on Intel and others */
}
/*
 * Use only for very small delays ( < 1 msec).  Should probably use a
 * lookup table, really, as the multiplications take much too long with
 * short delays.  This is a "reasonable" implementation, though (and the
 * first constant multiplications gets optimized away if the delay is
 * a constant)  
 */
static inline void udelay(unsigned long usecs)
{
	/* A loop takes 6 cycles on C6x */
	_c6x_delay((usecs * CONFIG_TMS320C6X_MHZ) / 6);
}

#define muldiv(a, b, c)    (((a)*(b))/(c))

#endif /* __ASM_C6X_DELAY_H */
