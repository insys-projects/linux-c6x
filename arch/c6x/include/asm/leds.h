/*
 *  arch/c6x/include/asm/leds.h
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
#ifndef __ASM_C6X_LEDS_H
#define __ASM_C6X_LEDS_H

#include <mach/board.h>

#ifndef __ASSEMBLY__
extern void c6x_arch_idle_led(int state);
#endif

#endif /* __ASM_C6X_LEDS_H */
