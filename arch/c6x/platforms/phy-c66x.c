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

int phy_init(void)
{
	return 0;
}

int evm_phy_init(void)
{
	struct sgmii_config_s sgmiic0, sgmiic1;

	sgmiic0.master    = 1;
	sgmiic0.loopback  = 0;
	sgmiic0.autoneg   = 0;
	sgmiic0.txconfig  = 0x000108a1;
	sgmiic0.rxconfig  = 0x00700621;
	sgmiic0.auxconfig = 0x00000041; /* PLL multiplier */

	c66x_sgmii_config(0, &sgmiic0);

	sgmiic1.master    = 1;
	sgmiic1.loopback  = 0;
	sgmiic1.autoneg   = 0;
	sgmiic1.txconfig  = 0x000108a1;
	sgmiic1.rxconfig  = 0x00700621;
	sgmiic1.auxconfig = 0x00000041; /* PLL multiplier */

	c66x_sgmii_config(1, &sgmiic1);

	phy_init();

	return 0;
}

arch_initcall(evm_phy_init);
