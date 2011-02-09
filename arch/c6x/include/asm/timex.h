/*
 *  linux/include/asm-c6x/timex.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010, 2011 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  Modified for 2.6.34: Mark Salter <msalter@redhat.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_TIMEX_H
#define __ASM_C6X_TIMEX_H

/*
 * This should be close enough...
 */
#define CLOCK_TICK_RATE ((1000 * 1000000UL) / 6)

#ifdef CONFIG_TMS320C64XPLUS
/* 64-bit timestamp */
typedef unsigned long long cycles_t;
#else
typedef unsigned long cycles_t;
#endif

extern cycles_t cacheflush_time;

static inline cycles_t get_cycles (void)
{
#ifdef CONFIG_TMS320C64XPLUS
	unsigned l, h;

#ifdef CONFIG_TI_C6X_COMPILER
	__dint();
	l = TSCL;
	h = TSCH;
	__rint();
#else
	asm volatile (" dint\n"
		      " mvc .s2 TSCL,%0\n"
		      " mvc .s2 TSCH,%1\n"
		      " rint\n"
		      : "=b"(l), "=b"(h));
#endif
	return ((cycles_t)h << 32) | l;
#else
	/* FIXME */
	return 0;
#endif
}

#ifdef CONFIG_TMS320C64XPLUS
extern int init_tsc_clocksource(void);
extern int init_timer64_clocksource(void);
#endif

#endif /* __ASM_C6X_TIMEX_H */
