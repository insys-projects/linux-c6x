/*
 *  linux/include/asm-c6x/sgmii.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2007, 2009, 2010 Texas Instruments Incorporated
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

extern int sgmii_reset(int port);
extern int sgmii_config(int port, struct sgmii_config_s *config);

#ifdef __KERNEL__
#include <asm/hardware.h>

#define SGMII_SRESET_RESET    0x1
#define SGMII_SRESET_RTRESET  0x2
#define SGMII_CTL_AUTONEG     0x01
#define SGMII_CTL_LOOPBACK    0x10
#define SGMII_CTL_MASTER      0x20

/*
 * SGMII registers
 */
#define SGMII_IDVER_REG(x)    (SGMII_REG_BASE + (x * 0x100) + 0x000)
#define SGMII_SRESET_REG(x)   (SGMII_REG_BASE + (x * 0x100) + 0x004)
#define SGMII_CTL_REG(x)      (SGMII_REG_BASE + (x * 0x100) + 0x010)
#define SGMII_STATUS_REG(x)   (SGMII_REG_BASE + (x * 0x100) + 0x014)
#define SGMII_MRADV_REG(x)    (SGMII_REG_BASE + (x * 0x100) + 0x018)
#define SGMII_LPADV_REG(x)    (SGMII_REG_BASE + (x * 0x100) + 0x020)
#define SGMII_TXCFG_REG(x)    (SGMII_REG_BASE + (x * 0x100) + 0x030)
#define SGMII_RXCFG_REG(x)    (SGMII_REG_BASE + (x * 0x100) + 0x034)
#define SGMII_AUXCFG_REG(x)   (SGMII_REG_BASE + (x * 0x100) + 0x038)

#define SGMII_REG_STATUS_FIELD_LOCK	        (1<<4)

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
