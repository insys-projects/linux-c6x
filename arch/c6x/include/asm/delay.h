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
 * Nanosecond delay for variable. Do not use division, only one multiplication 
 */
#define NSEC_FACTOR_SHIFT  6
#define NSEC_FACTOR_MULT   ((CONFIG_TMS320C6X_MHZ << NSEC_FACTOR_SHIFT) / (1000 * 6))
#define NSEC_CEIL          ((1 << NSEC_FACTOR_SHIFT) / NSEC_FACTOR_MULT)

static inline void __ndelay(unsigned long nsec)
{
	if (nsec > NSEC_CEIL)
		_c6x_delay((nsec * NSEC_FACTOR_MULT) >> NSEC_FACTOR_SHIFT);
}


/*
 * Nanosecond delay for small delays. Of course, it needs to be used with constant for small values.
 */
#define ndelay(n)            (__builtin_constant_p(n) ?			\
			      (n) <= NSEC_CEIL ? :			\
			      _c6x_delay(((n) * CONFIG_TMS320C6X_MHZ) / (1000 * 6)) : \
	                      __ndelay(n))

/*
 * Microsecond delay for variable. Do not use division, only one multiplication 
 */
#define USEC_FACTOR_SHIFT  2
#define USEC_FACTOR_MULT   ((CONFIG_TMS320C6X_MHZ << USEC_FACTOR_SHIFT) / 6)

static inline void __udelay(unsigned long usec)
{
	_c6x_delay((usec * USEC_FACTOR_MULT) >> USEC_FACTOR_SHIFT);
}

/*
 * Use only for very small delays ( < 1 msec).  Should probably use a
 * lookup table, really, as the multiplications take much too long with
 * short delays.  This is a "reasonable" implementation, though (and the
 * first constant multiplications gets optimized away if the delay is
 * a constant)  
 */
#define udelay(n)            (__builtin_constant_p(n) ?			\
			      _c6x_delay(((n) * CONFIG_TMS320C6X_MHZ) / 6) : \
			      __udelay(n))

#endif /* __ASM_C6X_DELAY_H */
