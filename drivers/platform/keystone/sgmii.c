/*
 * Copyright (C) 2011 Texas Instruments Incorporated
 * Authors: Sandeep Paulraj <s-paulraj@ti.com>
 *          Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <asm/setup.h>

#include <mach/keystone_qmss.h>
#include <mach/keystone_netcp.h>

#include <linux/keystone/sgmii.h>

/*
 * SGMII registers
 */
#define SGMII_IDVER_REG(x)              ((x * 0x100) + 0x000)
#define SGMII_SRESET_REG(x)             ((x * 0x100) + 0x004)
#define SGMII_CTL_REG(x)                ((x * 0x100) + 0x010)
#define SGMII_STATUS_REG(x)             ((x * 0x100) + 0x014)
#define SGMII_MRADV_REG(x)              ((x * 0x100) + 0x018)
#define SGMII_LPADV_REG(x)              ((x * 0x100) + 0x020)
#define SGMII_TXCFG_REG(x)              ((x * 0x100) + 0x030)
#define SGMII_RXCFG_REG(x)              ((x * 0x100) + 0x034)
#define SGMII_AUXCFG_REG(x)             ((x * 0x100) + 0x038)

#define SGMII_SRESET_RESET		0x01
#define SGMII_SRESET_RTRESET		0x02
#define SGMII_CTL_AUTONEG		0x01
#define SGMII_CTL_LOOPBACK		0x10
#define SGMII_CTL_MASTER		0x20
#define SGMII_REG_STATUS_FIELD_LOCK	(1<<4)

static void __iomem *sgmii_reg_base;

static inline void sgmii_write_reg(int reg, u32 val)
{
	__raw_writel(val, sgmii_reg_base + reg);
}

static inline u32 sgmii_read_reg(int reg)
{
	return __raw_readl(sgmii_reg_base + reg);
}

static inline void sgmii_write_reg_bit(int reg, u32 val)
{
	__raw_writel((__raw_readl(sgmii_reg_base + reg) | val),
		     sgmii_reg_base + reg);
}

int keystone_sgmii_reset(int port)
{
	sgmii_write_reg(SGMII_CTL_REG(port), 0);

	/* Soft reset */
	sgmii_write_reg_bit(SGMII_SRESET_REG(port), 0x1); 
	while(sgmii_read_reg(SGMII_SRESET_REG(port)) != 0x0);

	return 0;
}

int keystone_sgmii_config(int port, struct sgmii_config *config)
{
	unsigned int i, status;

	sgmii_write_reg(SGMII_CTL_REG(port), 0);

	sgmii_write_reg(SGMII_TXCFG_REG(port), config->txconfig);
	sgmii_write_reg(SGMII_RXCFG_REG(port), config->rxconfig);
	sgmii_write_reg(SGMII_AUXCFG_REG(port), config->auxconfig);

	/*
	 * Wait for the SerDes pll to lock,
	 * but don't trap if lock is never read
	 */
	for (i = 0; i < 1000; i++)  {
        	udelay(20000);
        	status = sgmii_read_reg(SGMII_STATUS_REG(port));
        	if ( (status & SGMII_REG_STATUS_FIELD_LOCK) != 0 )
			break;
	}

	sgmii_write_reg(SGMII_MRADV_REG(port), 1);
	sgmii_write_reg(SGMII_CTL_REG(port), 1);

	return 0;
}

int keystone_sgmii_init(void)
{
	sgmii_reg_base = ioremap(SGMII_REG_BASE, 0x150);
	return 0;
}
