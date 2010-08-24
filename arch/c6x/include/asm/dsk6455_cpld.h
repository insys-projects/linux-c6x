/*
 *  linux/include/asm-c6x/dsk6455_cpld.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2006, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_DSK6455_CPLD_H_
#define __ASM_C6X_DSK6455_CPLD_H_

#define DSK6455_CPLD_REG_BASE                    0xa0000000

/* DSK6455 CPLD registers */
#define DSK6455_CPLD_USER                        0x0
#define DSK6455_CPLD_DC                          0x1
#define DSK6455_CPLD_VERSION                     0x4
#define DSK6455_CPLD_MISC                        0x6
#define DSK6455_CPLD_MISC2                       0x7

#define cpld_setbit_reg(reg, val) \
        *((volatile unsigned char *) (DSK6455_CPLD_REG_BASE + (reg))) |= (unsigned char) (val)
	    
#define cpld_clearbit_reg(reg, val) \
        *((volatile unsigned char *) (DSK6455_CPLD_REG_BASE + (reg))) &= ~((unsigned char) (val))
        
#define cpld_set_reg(reg, val) \
        *((volatile unsigned char *) (DSK6455_CPLD_REG_BASE + (reg))) = (unsigned char) (val)
        
#define cpld_get_reg(reg) \
        *((volatile unsigned char *) (DSK6455_CPLD_REG_BASE + (reg)))

#endif /* __ASM_C6X_DSK6455_CPLD_H_ */
