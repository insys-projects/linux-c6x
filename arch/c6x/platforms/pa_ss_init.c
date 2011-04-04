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

int sgmii_init(void)
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

	printk("SGMII init complete\n");

	return 0;
}

int hw_cpsw_config(u32 ctl, u32 max_pkt_size)
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
	int i;
	
	sgmii_init();

	hw_cpsw_config(CPSW_CTL_P0_ENABLE, 9000);

//	hw_qm_setup((struct qm_config *)(target_get_qm_config()));
//	target_init_qs();

	i = target_pa_config();
	if (i != 0) {
		printk("\n PA init failed \n");
		return -1;
	}

	return 0;
}

arch_initcall(evm_pa_ss_init);
