/*
 *  linux/include/asm-c6x/sgmii.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2007, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@virtuallogix.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_SGMII_H_
#define __ASM_C6X_SGMII_H_

struct sgmii_config_s {
	unsigned int loopback;
	unsigned int master;
	unsigned int autoneg;
	unsigned int txconfig;
	unsigned int rxconfig;
	unsigned int auxconfig;
};

extern int sgmii_reset(void);
extern sgmii_config(struct sgmii_config_s *);

#ifdef __KERNEL__
#include <asm/hardware.h>

#define SGMII_SRESET_RESET    0x1
#define SGMII_SRESET_RTRESET  0x2
#define SGMII_CTL_AUTONEG     0x01
#define SGMII_CTL_LOOPBACK    0x10
#define SGMII_CTL_MASTER      0x20

#define sgmii_setbit_reg(reg, val) \
        *((volatile u32 *) (reg)) |= (u32) (val)
	    
#define sgmii_clearbit_reg(reg, val) \
        *((volatile u32 *) (reg)) &= ~((u32) (val))
        
#define sgmii_set_reg(reg, val) \
        *((volatile u32 *) (reg)) = (u32) (val)
        
#define sgmii_get_reg(reg) \
        *((volatile u32 *) (reg))

#define sgmii_addr_reg(reg) \
        ((volatile u32 *) (reg))

#endif /* __KERNEL__ */
#endif /* __ASM_C6X_SGMII_H_ */
