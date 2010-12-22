/*
 *  linux/arch/c6x/kernel/gmdio.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2006, 2009, 2010 Texas Instruments Incorporated
 *  Author: Nicolas Videau (nicolas.videau@jaluna.com)
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
#include <asm/gmdio.h>
#include <asm/pll.h>

static struct mdio_status mdios;

/*
 * Tick counts for timeout of each state
 * Note that NWAYSTART falls through to NWAYWAIT which falls through
 * to LINKWAIT. The timeout is not reset progressing from one state
 * to the next, so the system has 5 seconds total to find a link.
 */
static unsigned int phy_timeout[] = { 2,  /* MDIO_PHY_MDIOINIT  - min-delay  */
				      6,  /* MDIO_PHY_RESET     - .5 sec max */
				      41, /* MDIO_PHY_NWAYSTART - 4 seconds  */
				      41, /* MDIO_PHY_NWAYWAIT  - 4 seconds  */
				      51, /* MDIO_PHY_LINKWAIT  - 5 seconds  */
				      0 };/* MDIO_PHY_LINKED    - no timeout */

#ifdef CONFIG_SOC_TMS320C6455
/* ratio choices are limited to 2 and 5 */
#if 0
static unsigned int phy_clock_src[] = { PLLDIV_RATIO(5),    /* MII    */
					PLLDIV_RATIO(5),    /* RMII   */
					PLLDIV_RATIO(2),    /* GMII   */
					PLLDIV_RATIO(2) };  /* RGMII  */
#endif
#endif

static void mdio_init_state_machine(void)
{
	mdios.phy_addr    = 0;
	mdios.phy_state   = MDIO_PHY_MDIOINIT;
	mdios.phy_ticks   = 0;
	mdios.link_status = MDIO_LINKSTATUS_NOLINK;
}

/*
 * Initialize the MDIO
 */
int mdio_init(unsigned int txid_version)
{
#ifdef CONFIG_SOC_TMS320C6455
	/* Get MAC interface */
	mdios.macsel = (dscr_get_reg(DSCR_DEVSTAT) >> DEVSTAT_MACSEL_OFFSET) &
		DEVSTAT_MACSEL_MASK;
#if 0
	unsigned int ratio;

	/* Get MDIO source clock frequency */
	ratio = pll2_get_reg(PLLDIV1) & PLLDIV_RATIO_MASK;

	/* Adjust MDIO source clock frequency if needed */
	if ((mdios.macsel != DEVSTAT_MACSEL_MII) &&
	    (ratio != phy_clock_src[mdios.macsel])) {

	        /* Wait for last PLL GO command completion */
		pll2_wait_gostat();

		/* Set MDIO source clock to 25 or 125 MHz */
		pll2_set_reg(PLLDIV1,
			    phy_clock_src[mdios.macsel] | PLLDIV_EN);

		/* Launch a PLL GO command */
		pll2_set_goset();

		/* Wait for current PLL GO command completion */
		pll2_wait_gostat();
	}
#endif
#else
	/* Default (no gigabit support) */
	mdios.macsel = DEVSTAT_MACSEL_MII;
#endif

	/* Get Transmit identification and version */
	mdios.emac_txidver = txid_version;

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

/*
 * Return the current MDIO/PHY interface
 */
unsigned int mdio_get_macsel(void)
{
	mdios.macsel = (dscr_get_reg(DSCR_DEVSTAT) >> DEVSTAT_MACSEL_OFFSET) &
		DEVSTAT_MACSEL_MASK;

        return mdios.macsel;
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
static unsigned int mdio_init_phy_1(volatile unsigned int phy_addr)
{
	unsigned short   val, ack;
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
				       (MDIO_PHY_B_ISOLATE |
					MDIO_PHY_B_POWERDOWN));
			mdio_phy_wait();
		}

#ifdef CONFIG_LXT971
	/* Reset the PHY we plan to use */
	mdio_phy_write(MDIO_PHY_REG_CONTROL, phy_addr, MDIO_PHY_B_RESET);
	mdio_phy_wait_res_ack(i, ack);

	/* If the PHY did not ACK the write, return zero */
	if(!ack)
		return 0;
#endif

#ifdef CONFIG_ARCH_BOARD_DSK6455
	/* Settings for Broadcom phys */
	if (mdios.macsel == DEVSTAT_MACSEL_RGMII) {
		/* Put phy in copper mode */
		mdio_phy_write(MDIO_PHY_REG_ACCESS, phy_addr,
			       MDIO_PHY_B_COPPER);
		mdio_phy_wait_res_ack(val, ack);

		/* If the PHY did not ACK the write, return zero */
		if(!ack)
			return 0;

		mdio_phy_write(0x10, phy_addr, 0x0000);
		mdio_phy_wait();

		/* Put phy in RGMII mode/in-band status data for PG 2.0 */
		if (mdios.emac_txidver != 0x000C1207) {
			mdio_phy_write(MDIO_PHY_REG_SHADOW, phy_addr,
				       MDIO_PHY_B_INBAND);
			mdio_phy_wait_res_ack(val, ack);

			/* If the PHY did not ACK the write, return zero */
			if(!ack)
				return 0;
		}

		/* Patch for (older) board revB with R822 soldered */
		mdio_phy_write(MDIO_PHY_REG_ACCESS, phy_addr, 0x8C00);
		mdio_phy_wait_res_ack(val, ack);
		/* End of patch */
	}
#endif

#if 1
	if (mdios.macsel == DEVSTAT_MACSEL_GMII) {
		/* Put phy in copper mode */
		mdio_phy_write(MDIO_PHY_REG_ACCESS, phy_addr,
			       MDIO_PHY_B_COPPER);
		mdio_phy_wait_res_ack(val, ack);

		/* If the PHY did not ACK the write, return zero */
		if(!ack)
			return 0;
	}
#else
	/* Reset the PHY we plan to use */
	mdio_phy_write(MDIO_PHY_REG_CONTROL, phy_addr, MDIO_PHY_B_RESET);
	mdio_phy_wait();

	for (i = 0; i < 500; i++) {
		mdio_phy_read(MDIO_PHY_REG_CONTROL, phy_addr);
		mdio_phy_wait_res(val);
		if (!(val & MDIO_PHY_B_RESET))
			break;
	}
	if (i == 5000)
		return 0;
#endif

	/* Ready for next init step */
	mdios.phy_state   = MDIO_PHY_RESET;
	mdios.phy_ticks   = 0;

	return 1;
}

static unsigned int mdio_init_phy_2(volatile unsigned int phy_addr)
{
	unsigned short   val, val2, valgig = 0;

	mdios.phy_addr    = phy_addr;

	/* Read the STATUS reg to check autonegotiation capability */
	mdio_phy_read(MDIO_PHY_REG_STATUS, phy_addr);
	mdio_phy_wait_res(val);

	if ((mdios.macsel == DEVSTAT_MACSEL_GMII) ||
	    (mdios.macsel == DEVSTAT_MACSEL_RGMII)) {
		mdio_phy_read(MDIO_PHY_REG_EXTSTATUS, phy_addr);
		mdio_phy_wait_res(valgig);
	}

	/* See if we auto-neg or not */
	if ((mdios.mode & MDIO_MODE_AUTONEG) &&
	    (val & MDIO_PHY_B_AUTOCAPABLE)) {
		/* We will use NWAY */

		/* Advertise 1000 for supported interfaces */
		if ((mdios.macsel == DEVSTAT_MACSEL_GMII) ||
		    (mdios.macsel == DEVSTAT_MACSEL_RGMII)) {
			valgig >>= 4;
			valgig &= MDIO_PHY_ADV_FD1000;

			mdio_phy_write(MDIO_PHY_REG_1000CONTROL, phy_addr,
				       valgig);
			mdio_phy_wait();
		}

		/* Shift down the capability bits */
		val >>= 6;

		/* Mask with the capabilities */
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
			       (MDIO_PHY_B_AUTONEGEN | MDIO_PHY_B_AUTORESTART));
		mdio_phy_wait();

		/* Setup current state */
		mdios.mode |= MDIO_MODE_NWAYACTIVE;
		mdios.phy_state = MDIO_PHY_NWAYSTART;
		mdios.phy_ticks = 0;

	} else {
		/* Otherwise use a fixed configuration */

		/* Shift down the capability bits */
		val >>= 10;

		/* Mask with possible modes */
		val &= (MDIO_MODE_HD10 | MDIO_MODE_FD10 |
			MDIO_MODE_HD100 | MDIO_MODE_FD100);

		if ((mdios.macsel == DEVSTAT_MACSEL_GMII) ||
		    (mdios.macsel == DEVSTAT_MACSEL_RGMII)) {
			valgig >>= 8;
			valgig &= MDIO_MODE_FD1000;

			valgig &= mdios.mode;
		}

		/* Mask with what the User wants to allow */
		val &= mdios.mode;

		/* If nothing if left, move on */
		if((!val) && (!valgig))
			return 0;

		/* Setup Control word and pending status */
		if (valgig) {
			val2 = MDIO_PHY_B_SPEEDMSB | MDIO_PHY_B_DUPLEXFULL;
			mdios.pending_status = MDIO_LINKSTATUS_FD1000;
		} else if (val & MDIO_MODE_FD100) {
			val2 = MDIO_PHY_B_SPEEDLSB | MDIO_PHY_B_DUPLEXFULL;
			mdios.pending_status = MDIO_LINKSTATUS_FD100;
		} else if (val & MDIO_MODE_HD100) {
			val2 = MDIO_PHY_B_SPEEDLSB;
			mdios.pending_status = MDIO_LINKSTATUS_HD100;
		} else if (val & MDIO_MODE_FD10) {
			val2 = MDIO_PHY_B_DUPLEXFULL;
			mdios.pending_status = MDIO_PHY_B_FD10;
		} else {
			val2 = 0;
			mdios.pending_status = MDIO_LINKSTATUS_HD10;
		}

		/* Add in loopback if user wanted it */
		if (mdios.mode & MDIO_MODE_LOOPBACK )
			val2 |= MDIO_PHY_B_LOOPBACK;

		/* Configure PHY */
		mdio_phy_write(MDIO_PHY_REG_CONTROL, phy_addr,
			       (val2 | MDIO_PHY_B_AUTORESTART));
		mdio_phy_wait();

		/* Add in external loopback with plug if user wanted it */
		if (mdios.mode & MDIO_MODE_EXTLOOPBACK ) {
			mdio_phy_write(MDIO_PHY_REG_SHADOW, phy_addr,
				       MDIO_PHY_B_EXTLOOPBACK);
			mdio_phy_wait();
		}

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
	unsigned short val, val2, valgig = 0, valgig2 = 0, ack;
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
						       (MDIO_PHY_B_AUTONEGEN |
							MDIO_PHY_B_AUTORESTART));
					mdio_phy_wait();
				} else
					/* We have a Link - re-read NWAY params */
					mdios.phy_state = MDIO_PHY_NWAYWAIT;
			}
		}
	}

	if (mdios.phy_state != MDIO_PHY_LINKED) {
		/* Bump the time counter */
		mdios.phy_ticks++;

		/* Process differently based on state */
		switch(mdios.phy_state) {
#if !(defined(CONFIG_SOC_TMS3206472) || defined(CONFIG_SOC_TMS3206474)) && !defined(CONFIG_SOC_TMS3206457)
		case MDIO_PHY_RESET:
			/* Don't read reset status for the first 100 to 200 ms */
			if (mdios.phy_ticks < 2)
				break;

			/* See if the PHY has come out of reset */
			mdio_phy_read(MDIO_PHY_REG_CONTROL,
				      mdios.phy_addr);
			mdio_phy_wait_res_ack(val, ack);

			if (ack && !(val & MDIO_PHY_B_RESET)) {
				/* PHY is not reset.
				 * If the PHY init is going well, break out */
				if (mdio_init_phy_2(mdios.phy_addr))
					break;
				/* Else, this PHY is toast.
				 * Manually trigger a timeout */
				mdios.phy_ticks = phy_timeout[mdios.phy_state];
			}

			/* Fall through to timeout check */
#endif
		case MDIO_PHY_MDIOINIT:
 check_timeout:
			/* Here we just check timeout and try to find a PHY */
			if (mdios.phy_ticks >= phy_timeout[mdios.phy_state]) {
				/* Try the next PHY if anything but
				 * a MDIOINIT condition */
				if (mdios.phy_state != MDIO_PHY_MDIOINIT)
					if (++mdios.phy_addr == 32)
						mdios.phy_addr = 0;

#if (defined(CONFIG_TMS320DM643X) || defined(CONFIG_TMS320DM644X))
				lval = mdio_get_reg(MDIO_LINK);
#else
				lval = mdio_get_reg(MDIO_ALIVE);
#endif
				for (val = 0; val < 32; val++) {

					if ((lval & (1 << mdios.phy_addr)) &&
					    (mdio_init_phy_1(mdios.phy_addr)))
						break;

					if (++mdios.phy_addr == 32)
						mdios.phy_addr = 0;
				}

				/* If we didn't find a PHY, try again */
				if (val == 32) {
					mdios.phy_addr    = 0;
					mdios.phy_state   = MDIO_PHY_MDIOINIT;
					mdios.phy_ticks   = 0;
					res = MDIO_EVENT_PHYERROR;
				}
			}
			break;

		case MDIO_PHY_NWAYSTART:
			/* Start NWAY */

			/* Read the CONTROL reg to verify "restart" is not set */
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

			/* Fallthrough */

		case MDIO_PHY_NWAYWAIT:
			/* Waiting NWAY to complete */

			/* Read the STATUS reg to check for "complete" */
			mdio_phy_read(MDIO_PHY_REG_STATUS, mdios.phy_addr);
			mdio_phy_wait_res_ack(val, ack);
			if(!ack) {
				mdio_init_state_machine();
				break;
			}

			if (!(val & MDIO_PHY_B_AUTOCOMPLETE))
				goto check_timeout;
#if !(defined(CONFIG_SOC_TMS3206472) || defined(CONFIG_SOC_TMS3206474)) && !defined(CONFIG_SOC_TMS3206457)
			/* We can now check the negotiation results */
			if ((mdios.macsel == DEVSTAT_MACSEL_GMII) ||
			    (mdios.macsel == DEVSTAT_MACSEL_RGMII)) {
				mdio_phy_read(MDIO_PHY_REG_1000CONTROL,
					      mdios.phy_addr);
				mdio_phy_wait_res(valgig);
				mdio_phy_read(MDIO_PHY_REG_1000STATUS,
					      mdios.phy_addr);
				mdio_phy_wait_res(valgig2);
			}
#endif
			mdio_phy_read(MDIO_PHY_REG_ADVERTISE, mdios.phy_addr);
			mdio_phy_wait_res(val);
			mdio_phy_read(MDIO_PHY_REG_PARTNER, mdios.phy_addr);
			mdio_phy_wait_res(val2);

			val2 &= val;
#if !(defined(CONFIG_SOC_TMS3206472) || defined(CONFIG_SOC_TMS3206474)) && !defined(CONFIG_SOC_TMS3206457)
			if ((valgig & MDIO_PHY_ADV_FD1000) &&
			    (valgig2 & MDIO_PHY_PRT_FD1000))
				mdios.pending_status = MDIO_LINKSTATUS_FD1000;
			else if (val2 & MDIO_PHY_B_AFD100)
				mdios.pending_status = MDIO_LINKSTATUS_FD100;
#else
			if (val2 & MDIO_PHY_B_AFD100)
				mdios.pending_status = MDIO_LINKSTATUS_FD100;
#endif
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
