/*
 *  linux/arch/c6x/platforms/phy-evm6474.c
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

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/types.h>

#include <asm/setup.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/gmdio.h>
#include <asm/sgmii.h>

int phy_init(void) {
	int i;

	mdio_set_reg(MDIO_CONTROL, 0x4000001f); /* enable MII interface */

	_c6x_delay(145844);

	mdio_phy_write(4, 27, 0x81); /* turn PPU off to make copper PHY visible at SMI address 0x01 */
	mdio_phy_wait();

	_c6x_delay(8763);

	mdio_phy_write(26, 0xe, 0x47); /* set PHY port 6 SERDES to 0.7V swing */
	mdio_phy_wait();

	mdio_phy_write(26, 0xd, 0x47); /* set PHY port 5 SERDES to 0.7V swing */
	mdio_phy_wait();

	mdio_phy_write(0, 0xe, 0x8140); /* configure PHY port 6 SERDES --> Faraday 1 at 1000mpbs, full duplex */
	mdio_phy_wait();
	
	mdio_phy_write(0, 0xd, 0x8140); /* configure PHY port 5 SERDES --> Faraday 2 at 1000mbps, full duplex */
	mdio_phy_wait();

	mdio_phy_write(1, 0x15, 0x43e); /* force internal switch --> port 5 SERDES to 1000MPBS, full Duplex */
	mdio_phy_wait();

	mdio_phy_write(1, 0x16, 0x43e); /* force internal switch --> port 6 SERDES to 1000MBPS, full Duplex */
	mdio_phy_wait();

#if 0   /* Use autoneg at PHY level */

	/* force 1000 or 100mps at copper PHY, disable auto-negotiate */
	mdio_phy_write(0, 1, 0xa100); /* 0xa100 = 100mbps, 0x8140 = 1000mbps */
	mdio_phy_wait();

	/* wait for link establishment (~5 sec) */
	for (i = 0; i < 5000; i++)
		udelay(1000); /* 1ms */
#endif

	return 0;
}

int evm6474_phy_init(void) {
	struct sgmii_config_s sgmiic;

	/* SGMII setup */
	sgmii_reset();

	sgmiic.master    = 1;
	sgmiic.loopback  = 0;
	sgmiic.autoneg   = 0;
	sgmiic.txconfig  = 0x00000ea3; /* EVMC6474 board is wired up with TX differential +/- swapped. */
	sgmiic.rxconfig  = 0x00081023; /* programming serdes to be in master mode TX swapped */
	sgmiic.auxconfig = 0x0000000b; /* PLL multiplier */

	sgmii_config(&sgmiic);

	phy_init();

	return 0;
}

module_init(evm6474_phy_init);
