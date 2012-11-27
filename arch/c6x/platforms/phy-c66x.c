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
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/err.h>

#include <asm/setup.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/dscr.h>

#include <linux/keystone/sgmii.h>

static int serdes_init(void)
{
#ifdef CONFIG_ARCH_BOARD_EVM6670
	dscr_set_reg(DSCR_SGMII_SERDES_CFGPLL, 0x00000051);
#else
	dscr_set_reg(DSCR_SGMII_SERDES_CFGPLL, 0x00000041);
#endif
	_c6x_delay(2000);

#ifdef CONFIG_ARCH_BOARD_EVMTCI6616
	dscr_set_reg(DSCR_SGMII_SERDES_CFGRX0, 0x00460411);
	dscr_set_reg(DSCR_SGMII_SERDES_CFGRX1, 0x00460411);

	dscr_set_reg(DSCR_SGMII_SERDES_CFGTX0, 0x00011F91);
	dscr_set_reg(DSCR_SGMII_SERDES_CFGTX1, 0x00011F91);
#else
	dscr_set_reg(DSCR_SGMII_SERDES_CFGRX0, 0x00700621);
	dscr_set_reg(DSCR_SGMII_SERDES_CFGRX1, 0x00700621);

	dscr_set_reg(DSCR_SGMII_SERDES_CFGTX0, 0x000108A1);
	dscr_set_reg(DSCR_SGMII_SERDES_CFGTX1, 0x000108A1);
#endif
	_c6x_delay(2000);

	return 0;
}	

static int sgmii_init(void)
{
	struct sgmii_config sgmiic0, sgmiic1;

	/* SGMII to SGMII forced (AMC) */
	sgmiic0.master    = 1;
	sgmiic0.loopback  = 0;
	sgmiic0.autoneg   = 0;
#ifdef CONFIG_ARCH_BOARD_EVMTCI6616
	sgmiic0.txconfig  = 0x00011f91;
	sgmiic0.rxconfig  = 0x00460411;
#else
	sgmiic0.txconfig  = 0x000108a1;
	sgmiic0.rxconfig  = 0x00700621;
#endif
#ifdef CONFIG_ARCH_BOARD_EVM6670
	sgmiic0.auxconfig = 0x00000051; /* PLL multiplier */
#else
	sgmiic0.auxconfig = 0x00000041; /* PLL multiplier */
#endif
	keystone_sgmii_config(0, &sgmiic0);

	/* SGMII to PHY (RJ45) */
	sgmiic1.master    = 0;
	sgmiic1.loopback  = 0;
	sgmiic1.autoneg   = 1;
#ifdef CONFIG_ARCH_BOARD_EVMTCI6616
	sgmiic1.txconfig  = 0x00011f91;
	sgmiic1.rxconfig  = 0x00460411;
#else
	sgmiic1.txconfig  = 0x000108a1;
	sgmiic1.rxconfig  = 0x00700621;
#endif
#ifdef CONFIG_ARCH_BOARD_EVM6670
	sgmiic1.auxconfig = 0x00000051; /* PLL multiplier */
#else
	sgmiic1.auxconfig = 0x00000041; /* PLL multiplier */
#endif
	keystone_sgmii_config(1, &sgmiic1);

	printk("SGMII init complete\n");

	return 0;
}

int evm_phy_init(void)
{	
	if (get_coreid() == get_master_coreid()) {

		/* Initialize SGMII driver */
		keystone_sgmii_init();

		/* Reset SGMII */
		keystone_sgmii_reset(0);
		keystone_sgmii_reset(1);

		/* SERDES init */
		serdes_init();
		
		/* Configure the SGMII */
		sgmii_init();
	}

	return 0;
}
arch_initcall(evm_phy_init);
