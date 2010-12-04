/*
 *  linux/include/asm-c6x/hardware.h
 *
 *  Port on Texas Instruments TMS320C6x/C6x+ architecture
 *
 *  Copyright (C) 2005, 2006, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_HARDWARE_H_
#define __ASM_C6X_HARDWARE_H_

#include <mach/hardware.h>

#define __SYSREG(ADDR, TYPE) (*(volatile TYPE*)(ADDR))
#define __SYSREGA(ADDR, TYPE) ((volatile TYPE*)(ADDR))

#ifndef __ASSEMBLY__
extern void c6x_soc_setup_arch(void);
extern void c6x_board_setup_arch(void);
extern char *arch_compute_silicon_rev(u32 silicon_rev);
extern unsigned int arch_get_silicon_rev(void);
#endif

#endif /* __ASM_C6X_HARDWARE_H_ */
