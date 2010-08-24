/*
 *  linux/arch/c6x/drivers/mdio.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/spinlock.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/mdio.h>

static struct mdio_status mdios;

/*
 * Tick counts for timeout of each state
 * Note that NWAYSTART falls through to NWAYWAIT which falls through
 * to LINKWAIT. The timeout is not reset progressing from one state
 * to the next, so the system has 5 seconds total to find a link.
 */
static unsigned int phy_timeout[] = { 2,  /* MDIO_PHY_MDIOINIT   - min-delay */
				      41, /* MDIO_PHY_NWAYSTART  - 4 seconds */
				      41, /* MDIO_PHY_NWAYWAIT   - 4 seconds */
				      51, /* MDIO_PHY_LINKWAIT   - 5 seconds */
				      0 };/* MDIO_PHY_LINKED     - no timeout*/

static void mdio_init_state_machine(void)
{
	mdios.phy_state   = MDIO_PHY_MDIOINIT;
	mdios.phy_ticks   = 0;
	mdios.phy_addr    = 0;
	mdios.link_status = MDIO_LINKSTATUS_NOLINK;
}

/*
 * Initialize the MDIO
 */
int mdio_init(void)
{
	mdios.mode = MDIO_MODE_AUTONEG; /* autonegotiate */
	mdio_init_state_machine();
	mdio_set_reg(MDIO_CONTROL, EMAC_B_ENABLE | (VBUSCLK & EMAC_M_CLKDIV));
	return 0;
}

/*
 * Return the current MDIO/PHY status
 */
unsigned int mdio_get_status(void)
{
        return mdios.link_status;
}

#ifdef CONFIG_LSI_LOGIC
/*
 * Link status change of LSI LOGIC transceiver
 */
void mdio_lsi_linkstatus(unsigned int status)
{
	unsigned short val = 0, ack = 0;

	mdio_phy_read(17, mdios.phy_addr);
	mdio_phy_wait_res_ack(val, ack);

	if (ack) {
		val &= 0xff3f;

		if (status == MDIO_LINKSTATUS_FD100)
			val |= (1 << 6);   /* green led on */

		if (status == MDIO_LINKSTATUS_FD10)
			val &= ~(1 <<6);   /* green led off */

		mdio_phy_write(17, mdios.phy_addr, val);
		mdio_phy_wait_res_ack(val, ack);
	}
}
#endif

/*
 * Force a switch to the specified PHY, and start the negotiation process
 */
static unsigned int mdio_init_phy(volatile unsigned int phy_addr)
{
	unsigned short   val,val2;
	unsigned int     lval;
	unsigned int     i;

	mdios.phy_addr    = phy_addr;
	mdios.link_status = MDIO_LINKSTATUS_NOLINK;

	/* Shutdown all other PHYs */
	lval = mdio_get_reg(MDIO_ALIVE);
	for (i = 0; lval; i++,lval >>= 1)
		if((lval & 1) && (i != phy_addr)) {
			mdio_phy_write(MDIO_PHY_REG_CONTROL,
				       i,
				       MDIO_PHY_B_ISOLATE |
				       MDIO_PHY_B_POWERDOWN);
			mdio_phy_wait();
		}

#ifdef CONFIG_LXT971
	/* Reset the PHY we plan to use */
	mdio_phy_write(MDIO_PHY_REG_CONTROL, phy_addr, MDIO_PHY_B_RESET);
	mdio_phy_wait();

	val = MDIO_PHY_B_RESET;
	/* Wait for reset to go low (but not forever) */
	for (i = 0; (i < 5000) && (val & MDIO_PHY_B_RESET); i++) {
		mdio_phy_read(MDIO_PHY_REG_CONTROL, phy_addr);
		mdio_phy_wait_res(val);
	}
	if (i == 5000)
		return 0;
#endif

	/* Read the STATUS reg to check autonegotiation capability */
	mdio_phy_read(MDIO_PHY_REG_STATUS, phy_addr);
	mdio_phy_wait_res(val);

	/* See if we auto-neg or not */
	if((mdios.mode & MDIO_MODE_AUTONEG) &&
	   (val & MDIO_PHY_B_AUTOCAPABLE)) {

		val >>= 6;
		val &= (MDIO_PHY_B_AFD100 | MDIO_PHY_B_AHD100 |
			MDIO_PHY_B_AFD10 | MDIO_PHY_B_AHD10);

		/* Set Ethernet message bit */
		val |= MDIO_PHY_B_MSG;

		/* Write out advertisement */
		mdio_phy_write(MDIO_PHY_REG_ADVERTISE, phy_addr, val);
		mdio_phy_wait();

		/* Start NWAY */
		mdio_phy_write(MDIO_PHY_REG_CONTROL,
			       phy_addr,
			       MDIO_PHY_B_AUTONEGEN);
		mdio_phy_wait();

		mdio_phy_write(MDIO_PHY_REG_CONTROL,
			       phy_addr,
			       MDIO_PHY_B_AUTONEGEN | MDIO_PHY_B_AUTORESTART);
		mdio_phy_wait();

		/* Setup current state */
		mdios.mode |= MDIO_MODE_NWAYACTIVE;
		mdios.phy_state = MDIO_PHY_NWAYSTART;
		mdios.phy_ticks = 0;
	} else {
		/* Otherwise use a fixed configuration */

		val >>= 10;
		val &= (MDIO_MODE_HD10 | MDIO_MODE_FD10 |
			MDIO_MODE_HD100 | MDIO_MODE_FD100);

		val &= mdios.mode;
		if(!val)
			return 0;

		/* Setup Control word and pending status */
		if (val & MDIO_MODE_FD100) {
			val2 = MDIO_PHY_B_SPEED100 | MDIO_PHY_B_DUPLEXFULL;
			mdios.pending_status = MDIO_PHY_B_FD100;
		} else if (val & MDIO_MODE_HD100) {
			val2 = MDIO_PHY_B_SPEED100;
			mdios.pending_status = MDIO_PHY_B_HD100;
		} else if (val & MDIO_MODE_FD10) {
			val2 = MDIO_PHY_B_DUPLEXFULL;
			mdios.pending_status = MDIO_PHY_B_FD10;
		} else {
			val2 = 0;
			mdios.pending_status = MDIO_PHY_B_HD10;
		}

		/* Add in loopback if user wanted it */
		if (mdios.mode & MDIO_MODE_LOOPBACK )
			val2 |= MDIO_PHY_B_LOOPBACK;

		/* Configure PHY */
		mdio_phy_write(MDIO_PHY_REG_CONTROL, phy_addr, val2);
		mdio_phy_wait();

		/* Setup current state */
		mdios.mode &= ~MDIO_MODE_NWAYACTIVE;
		mdios.phy_state = MDIO_PHY_LINKWAIT;
		mdios.phy_ticks = 0;
    }
    return 1;
}


/*
 * Called each 100mS to check if MDIO status has changed.
 * A MDIO event is returned.
 */
unsigned int mdio_timer_tick(void)
{
	unsigned int res = MDIO_EVENT_NOCHANGE;
	unsigned short val, val2, ack;
	unsigned int lval;

	if (mdios.phy_state == MDIO_PHY_LINKED) {
		/*
		 * Check for a "link-change" status indication or a link
		 * down indication.
		 */
		lval = mdio_get_reg(MDIO_LINKINTRAW) & 1;
		mdio_set_reg(MDIO_LINKINTRAW, lval);
		if (lval || !(mdio_get_reg(MDIO_LINK) & (1 << mdios.phy_addr))) {

			mdios.link_status = MDIO_LINKSTATUS_NOLINK;
			mdios.phy_ticks = 0;
			res = MDIO_EVENT_LINKDOWN;

			/* If not NWAY, just wait for link */
			if (!(mdios.mode & MDIO_MODE_NWAYACTIVE))
				mdios.phy_state = MDIO_PHY_LINKWAIT;
			else {
				/* Handle NWAY condition */
				mdio_phy_read(MDIO_PHY_REG_STATUS,
					      mdios.phy_addr);
				mdio_phy_wait();
				mdio_phy_read(MDIO_PHY_REG_STATUS,
					      mdios.phy_addr);
				mdio_phy_wait_res_ack(val, ack);
				if(!ack)
					mdio_init_state_machine();

				else if(!(val & MDIO_PHY_B_LINKSTATUS)) {
					/* No Link - restart NWAY */
					mdios.phy_state = MDIO_PHY_NWAYSTART;

					mdio_phy_write(MDIO_PHY_REG_CONTROL,
						       mdios.phy_addr,
						       MDIO_PHY_B_AUTONEGEN |
						       MDIO_PHY_B_AUTORESTART);
					mdio_phy_wait();
				} else
					mdios.phy_state = MDIO_PHY_NWAYWAIT;
			}
		}
	}

	if (mdios.phy_state != MDIO_PHY_LINKED) {
		mdios.phy_ticks++;
		switch(mdios.phy_state) {

		case MDIO_PHY_MDIOINIT:
 check_timeout:
			if (mdios.phy_ticks >= phy_timeout[mdios.phy_state]) {
				if (mdios.phy_state != MDIO_PHY_MDIOINIT)
					if (++mdios.phy_addr == 32)
						mdios.phy_addr = 0;

				lval = mdio_get_reg(MDIO_ALIVE);
				for (val = 0; val < 32; val++) {

					if ((lval & (1 << mdios.phy_addr)) &&
					    (mdio_init_phy(mdios.phy_addr)))
						break;

					if (++mdios.phy_addr == 32)
						mdios.phy_addr = 0;
				}

				if (val == 32) {
					mdios.phy_state   = MDIO_PHY_MDIOINIT;
					mdios.phy_ticks   = 0;
					mdios.phy_addr    = 0;
					res = MDIO_EVENT_PHYERROR;
				}
			}
			break;

		case MDIO_PHY_NWAYSTART:
			/* Start NWAY */
			mdio_phy_read(MDIO_PHY_REG_CONTROL, mdios.phy_addr);
			mdio_phy_wait_res_ack(val, ack);
			if (!ack) {
				mdio_init_state_machine();
				break;
			}

			if (val & MDIO_PHY_B_AUTORESTART)
				goto check_timeout;

			/* Flush latched "link status" from the STATUS reg */
			mdio_phy_read(MDIO_PHY_REG_STATUS, mdios.phy_addr);
			mdio_phy_wait();

			mdios.phy_state = MDIO_PHY_NWAYWAIT;

		case MDIO_PHY_NWAYWAIT:
			/* Waiting NWAY to complete */
			mdio_phy_read(MDIO_PHY_REG_STATUS, mdios.phy_addr);
			mdio_phy_wait_res_ack(val, ack);
			if(!ack) {
				mdio_init_state_machine();
				break;
			}

			if (!(val & MDIO_PHY_B_AUTOCOMPLETE))
				goto check_timeout;

			/* We can now check the negotiation results */
			mdio_phy_read(MDIO_PHY_REG_ADVERTISE, mdios.phy_addr);
			mdio_phy_wait_res(val);
			mdio_phy_read(MDIO_PHY_REG_PARTNER, mdios.phy_addr);
			mdio_phy_wait_res(val2);

			val2 &= val;

			if (val2 & MDIO_PHY_B_AFD100)
				mdios.pending_status = MDIO_LINKSTATUS_FD100;
			else if( val2 & MDIO_PHY_B_AHD100)
				mdios.pending_status = MDIO_LINKSTATUS_HD100;
			else if( val2 & MDIO_PHY_B_AFD10)
				mdios.pending_status = MDIO_LINKSTATUS_FD10;
			else if( val2 & MDIO_PHY_B_AHD10)
				mdios.pending_status = MDIO_LINKSTATUS_HD10;
			else if( val & MDIO_PHY_B_AHD100)
				mdios.pending_status = MDIO_LINKSTATUS_HD100;
			else
				mdios.pending_status = MDIO_LINKSTATUS_HD10;

			mdios.phy_state = MDIO_PHY_LINKWAIT;

		case MDIO_PHY_LINKWAIT:
			/* Waiting for LINK */
			mdio_phy_read(MDIO_PHY_REG_STATUS, mdios.phy_addr);
			mdio_phy_wait_res_ack(val, ack);
			if(!ack) {
				mdio_init_state_machine();
				break;
			}

			if(!(val & MDIO_PHY_B_LINKSTATUS))
				goto check_timeout;

			/* Make sure we're linked in the MDIO module as well */
			lval = mdio_get_reg(MDIO_LINK);
			if(!(lval & (1 << mdios.phy_addr)) )
				goto check_timeout;

			/* Start monitoring this PHY */
			mdio_set_reg(MDIO_USERPHYSEL0, mdios.phy_addr);

			/* Clear the link change flag so we can detect a "re-link" later */
			mdio_set_reg(MDIO_LINKINTRAW, 1);

			/* Setup our linked state */
			mdios.phy_state   = MDIO_PHY_LINKED;
			mdios.link_status = mdios.pending_status;
			res = MDIO_EVENT_LINKUP;
			break;
		}
	}
	return res;
}
