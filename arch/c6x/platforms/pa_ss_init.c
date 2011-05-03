/*
 * Copyright (C) 2011 Texas Instruments Incorporated
 * Author: Sandeep Paulraj <s-paulraj@ti.com>
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
#include <linux/delay.h>
#include <linux/types.h>

#include <asm/setup.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/gmdio.h>
#include <asm/sgmii.h>

#include <mach/netcp.h>
#include <mach/pa.h>
#include <mach/keystone_qmss.h>

#define KICK0		(BOOTCFG_BASE + 0x0038)
#define KICK1		(BOOTCFG_BASE + 0x003C)
#define KICK0_UNLOCK	(0x83E70B13)
#define KICK1_UNLOCK	(0x95A4F1E0)
#define KICK_LOCK	0

#define BOOTCFG_BASE		0x02620000
#define SGMII_SERDES_CFGPLL	(BOOTCFG_BASE + 0x340)
#define SGMII_SERDES_CFGRX0	(BOOTCFG_BASE + 0x344)
#define SGMII_SERDES_CFGTX0	(BOOTCFG_BASE + 0x348)
#define SGMII_SERDES_CFGRX1	(BOOTCFG_BASE + 0x34C)
#define SGMII_SERDES_CFGTX1	(BOOTCFG_BASE + 0x350)

static int sgmii_init(void)
{
	struct sgmii_config_s sgmiic0, sgmiic1;

	sgmiic0.master    = 1;
	sgmiic0.loopback  = 0;
	sgmiic0.autoneg   = 0;
	sgmiic0.txconfig  = 0x000108a1;
	sgmiic0.rxconfig  = 0x00700621;
#ifdef CONFIG_ARCH_BOARD_EVM6670
	sgmiic0.auxconfig = 0x00000051; /* PLL multiplier */
#else
	sgmiic0.auxconfig = 0x00000041; /* PLL multiplier */
#endif
	c66x_sgmii_config(0, &sgmiic0);

	sgmiic1.master    = 1;
	sgmiic1.loopback  = 0;
	sgmiic1.autoneg   = 0;
	sgmiic1.txconfig  = 0x000108a1;
	sgmiic1.rxconfig  = 0x00700621;
#ifdef CONFIG_ARCH_BOARD_EVM6670
	sgmiic0.auxconfig = 0x00000051; /* PLL multiplier */
#else
	sgmiic1.auxconfig = 0x00000041; /* PLL multiplier */
#endif
	
	c66x_sgmii_config(1, &sgmiic1);

	printk("SGMII init complete\n");

	return 0;
}

static int hw_cpsw_config(u32 ctl, u32 max_pkt_size)
{
	u32 i;

	/* Max length register */
	__raw_writel(max_pkt_size, (DEVICE_CPSW_BASE + CPSW_REG_MAXLEN));
	
	/* Control register */
	__raw_writel(ctl, (DEVICE_CPSW_BASE + CPSW_REG_CTL));
	
	/* All statistics enabled by default */
	__raw_writel(CPSW_REG_VAL_STAT_ENABLE_ALL, (DEVICE_CPSW_BASE +
						    CPSW_REG_STAT_PORT_EN));
	
	/* Reset and enable the ALE */
	__raw_writel(CPSW_REG_VAL_ALE_CTL_RESET_AND_ENABLE, (DEVICE_CPSW_BASE
							     + CPSW_REG_ALE_CONTROL));
    
	/* All ports put into forward mode */
	for (i = 0; i < DEVICE_CPSW_NUM_PORTS; i++)
		__raw_writel(CPSW_REG_VAL_PORTCTL_FORWARD_MODE,
			     (DEVICE_CPSW_BASE + CPSW_REG_ALE_PORTCTL(i)));
	
	return 0;
}

int evm_pa_ss_init(void)
{	
	/* SERDES init */

	__raw_writel(KICK0_UNLOCK, KICK0);
	__raw_writel(KICK1_UNLOCK, KICK1);
	
	__raw_writel(0x00000041, SGMII_SERDES_CFGPLL);

	_c6x_delay(2000);

	__raw_writel(0x00700621, SGMII_SERDES_CFGRX0);
	__raw_writel(0x00700621, SGMII_SERDES_CFGRX1);

	__raw_writel(0x000108A1, SGMII_SERDES_CFGTX0);
	__raw_writel(0x000108A1, SGMII_SERDES_CFGTX1);

	_c6x_delay(2000);

	__raw_writel(KICK_LOCK, KICK0);
	__raw_writel(KICK_LOCK, KICK1);

	
	/* Configure the SGMII */
	sgmii_init();

	/* Enable port 0 with max pkt size to 9000 */
	hw_cpsw_config(CPSW_CTL_P0_ENABLE, 9000);

	return 0;
}

arch_initcall(evm_pa_ss_init);
