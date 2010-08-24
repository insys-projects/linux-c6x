/*
 *  linux/include/asm-c6x/timer.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2005, 2006, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_TIMER_H
#define __ASM_C6X_TIMER_H

#include <linux/interrupt.h>
#ifdef CONFIG_NK
#include <asm/nkern.h>
#endif
#include <mach/timer.h>

#ifdef CONFIG_USE_RT_TIMER
extern void adjust_time(void);
#endif

#define TIMER_REG(reg)       (*((volatile unsigned int*) (reg)))
#define TIMER_REG64(reg)     (*((volatile unsigned long long *) (reg)))

#ifndef __ASSEMBLY__
extern int c6x_arch_init_clocksource(void);
extern int c6x_arch_init_clockevents(void);
#endif  /* __ASSEMBLY__ */

#endif /*__ASM_C6X_TIMER_H */
