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

extern unsigned int ticks_per_ns_scaled;

static inline void __delay(unsigned long loop)
{
	_c6x_delay(loop / 3); /* Because a loop takes 6 cycles on C6x instead
				 of 2 on Intel and others */
}

#ifdef CONFIG_TI_C6X_COMPILER
extern asmlinkage void _c6x_tickdelay(unsigned int);
#else
static inline void _c6x_tickdelay(unsigned int x)
{
	uint32_t cnt, endcnt;
	asm volatile (" mvc .s2 TSCL,%0\n"
		      " add .s2x %0,%1,%2\n"
		      " || mvk .l2 1,B0\n"
		      "0: [B0] b .s2 0b\n"
		      " mvc .s2 TSCL,%0\n"
		      " sub .s2 %0,%2,%0\n"
		      " cmpgt .l2 0,%0,B0\n"
		      " nop\n"
		      " nop\n"
		      : "=b"(cnt), "+a"(x), "=b"(endcnt) : : "B0");
}
#endif

/* use scaled math to avoid slow division */
#define C6X_NDELAY_SCALE 10

static inline void _ndelay(unsigned int n)
{
	_c6x_tickdelay((ticks_per_ns_scaled * n) >> C6X_NDELAY_SCALE);
}

static inline void _udelay(unsigned int n)
{
	while (n >= 10) {
		_ndelay(10000);
		n -= 10;
	}
	while (n-- > 0)
		_ndelay(1000);
}

#define udelay(x) _udelay((unsigned int)(x))
#define ndelay(x) _ndelay((unsigned int)(x))

#endif /* __ASM_C6X_DELAY_H */
