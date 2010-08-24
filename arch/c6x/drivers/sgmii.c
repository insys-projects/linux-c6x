/*
 *  linux/arch/c6x/drivers/sgmii.c
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
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/netdevice.h>

#include <asm/setup.h>
#include <asm/machdep.h>
#include <asm/sgmii.h>
#include <asm/gemac.h>

int sgmii_reset(void)
{
	sgmii_setbit_reg(SGMII_SRESET_REG, SGMII_SRESET_RESET); /* soft reset */
	while(sgmii_get_reg(SGMII_SRESET_REG) != 0x0);
	sgmii_setbit_reg(SGMII_CTL_REG, SGMII_CTL_MASTER);

	return 0;
}

int sgmii_config(struct sgmii_config *config)
{
	unsigned int i, val1 = 0, val2 = 0;

	if (!config)
		return -1;

	if (config->loopback) {
		val1 = SGMII_CTL_MASTER | SGMII_CTL_LOOPBACK | SGMII_CTL_AUTONEG; /* 0x35 */
		val2 = 0x9801;
	}

	if (config->master) {
		val1 = SGMII_CTL_MASTER;
		val2 = 0x9801;   /* advertise fullduplex gigabit */
	} else {
		val1 = 0;
		val2 = 0x01;     /* advertise fullduplex gigabit */
	}

	if (config->autoneg)
		val1 |= SGMII_CTL_AUTONEG;

	sgmii_set_reg(SGMII_SRESET_REG, SGMII_SRESET_RTRESET); /* RT soft reset */
	sgmii_set_reg(SGMII_CTL_REG, val1);
	sgmii_clearbit_reg(SGMII_SRESET_REG, SGMII_SRESET_RTRESET);

	sgmii_set_reg(SGMII_MRADV_REG, val2);
#ifndef CONFIG_TMS320DM648
	sgmii_set_reg(SGMII_TXCFG_REG, config->txconfig);
	sgmii_set_reg(SGMII_RXCFG_REG, config->rxconfig);
	sgmii_set_reg(SGMII_AUXCFG_REG, config->auxconfig);
#else
	sgmii_set_reg(SERDES_KEY_REG, SERDES_KEY_VAL);
	sgmii_set_reg(SERDES_PLL_REG, config->auxconfig);
	for (i=0; i<2500; i++)
		asm("    nop");
	sgmii_set_reg(SERDES_RX0_REG, config->rxconfig);
	sgmii_set_reg(SERDES_TX0_REG, config->txconfig);
#endif
	if (config->autoneg) {
		unsigned int done = 0x5;
		unsigned int stat;

		/* Wait for auto-negotiation*/
		do {
			stat = sgmii_get_reg(SGMII_STATUS_REG);
			stat &= done;
		} while (stat != done);
	}

	return 0;
}


