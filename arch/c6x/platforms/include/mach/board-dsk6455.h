/*
 *  linux/arch/c6x/platforms/mach/board-dsk6455.h
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

/* board memory map */
#define VECTADDR        0xE0000000
#define TEXTADDR	0xE0000400
#define TEXTLEN         0x07FFFC00
#define RAMEND		0xE8000000

/*
 * Interrupt Assignments
 */
#define IRQ_CLOCKEVENTS INT15
#define IRQ_EMAC        INT6



#define DSK6455_CPLD_REG_BASE                    0xa0000000

/* DSK6455 CPLD registers */
#define DSK6455_CPLD_USER                        0x0
#define DSK6455_CPLD_DC                          0x1
#define DSK6455_CPLD_VERSION                     0x4
#define DSK6455_CPLD_MISC                        0x6
#define DSK6455_CPLD_MISC2                       0x7

#ifndef __ASSEMBLY__
#define cpld_setbit_reg(reg, val) \
        *((volatile unsigned char *) (DSK6455_CPLD_REG_BASE + (reg))) |= (unsigned char) (val)
	    
#define cpld_clearbit_reg(reg, val) \
        *((volatile unsigned char *) (DSK6455_CPLD_REG_BASE + (reg))) &= ~((unsigned char) (val))
        
#define cpld_set_reg(reg, val) \
        *((volatile unsigned char *) (DSK6455_CPLD_REG_BASE + (reg))) = (unsigned char) (val)
        
#define cpld_get_reg(reg) \
        *((volatile unsigned char *) (DSK6455_CPLD_REG_BASE + (reg)))


#ifdef CONFIG_IDLE_LED
static inline void c6x_arch_idle_led(int state)
{
	if (state)
		cpld_setbit_reg(DSK6455_CPLD_USER,1);
	else
		cpld_clearbit_reg(DSK6455_CPLD_USER,1);
}
#endif
#endif
