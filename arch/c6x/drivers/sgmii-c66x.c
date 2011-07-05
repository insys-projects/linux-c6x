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
#include <linux/netdevice.h>
#include <linux/delay.h>

#include <asm/setup.h>
#include <asm/machdep.h>
#include <asm/sgmii.h>

#include <mach/keystone_netcp.h>

extern int sgmii_reset(int port)
{
	sgmii_set_reg(SGMII_CTL_REG(port), 0);

	/* Soft reset */
	sgmii_setbit_reg(SGMII_SRESET_REG(port), 0x1); 
	while(sgmii_get_reg(SGMII_SRESET_REG(port)) != 0x0);

	return 0;
}

int sgmii_config(int port, struct sgmii_config_s *config)
{
	unsigned int i, status;

	sgmii_set_reg(SGMII_CTL_REG(port), 0);

	sgmii_set_reg(SGMII_TXCFG_REG(port), config->txconfig);
	sgmii_set_reg(SGMII_RXCFG_REG(port), config->rxconfig);
	sgmii_set_reg(SGMII_AUXCFG_REG(port), config->auxconfig);

	/*
	 * Wait for the SerDes pll to lock,
	 * but don't trap if lock is never read
	 */
	for (i = 0; i < 1000; i++)  {
        	_c6x_delay(2000); 
        	status = sgmii_get_reg(SGMII_STATUS_REG(port));
        	if ( (status & SGMII_REG_STATUS_FIELD_LOCK) != 0 )
			break;
	}

	sgmii_set_reg(SGMII_MRADV_REG(port), 1);
	sgmii_set_reg(SGMII_CTL_REG(port), 1);

	return 0;
}


