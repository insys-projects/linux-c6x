/*
 *  linux/include/asm-c6x/mdio.h
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
#ifndef __ASM_C6X_MDIO_H_
#define __ASM_C6X_MDIO_H_

#ifdef __KERNEL__
#include <asm/dscr.h>

#define MDIO_VERSION              0x000 /* Module Version Register */
#define MDIO_CONTROL              0x004 /* Module Control Register */
#define MDIO_ALIVE                0x008 /* PHY "Alive" Indication Register */
#define MDIO_LINK                 0x00c /* PHY Link Status Register */
#define MDIO_LINKINTRAW           0x010 /* Link Status Change Interrupt Register */
#define MDIO_LINKINTMASKED        0x014 /* Link Status Change Interrupt Register (Masked) */
#define MDIO_USERINTRAW           0x020 /* User Command Complete Interrupt */
#define MDIO_USERINTMASKED        0x024 /* User Command Complete Interrupt (Masked) */
#define MDIO_USERINTMASKSET       0x028 /* Enable User Command Complete Interrupt Mask */
#define MDIO_USERINTMASKCLEAR     0x02c /* Disable User Command Complete Interrupt Mask */
#define MDIO_USERACCESS0          0x080 /* User Access Register 0 */
#define MDIO_USERPHYSEL0          0x084 /* User PHY Select Register 0 */
#define MDIO_USERACCESS1          0x088 /* User Access Register 1 */
#define MDIO_USERPHYSEL1          0x08c /* User PHY Select Register 1 */

#define MDIO_M_CLKDIV            ((1 << 16) - 1)

#define MDIO_B_FAULTENB          (1 << 18)
#define MDIO_B_FAULT             (1 << 19)
#define MDIO_B_PREAMBLE          (1 << 20)
#define MDIO_B_ENABLE            (1 << 30)
#define MDIO_B_IDLE              (1 << 31)

#define MDIO_B_ACK               (1 << 29)
#define MDIO_B_WRITE             (1 << 30)
#define MDIO_B_GO                (1 << 31) /* for USERACESS */

#define mdio_setbit_reg(reg, val) \
        *((volatile u32 *) (MDIO_REG_BASE + (reg))) |= (u32) (val)
	    
#define mdio_clearbit_reg(reg, val) \
        *((volatile u32 *) (MDIO_REG_BASE + (reg))) &= ~((u32) (val))
        
#define mdio_set_reg(reg, val) \
        *((volatile u32 *) (MDIO_REG_BASE + (reg))) = (u32) (val)
        
#define mdio_get_reg(reg) \
        *((volatile u32 *) (MDIO_REG_BASE + (reg)))

#define mdio_addr_reg(reg) \
        ((volatile u32 *) (MDIO_REG_BASE + (reg)))


/*
 * MDIO status
 */
struct mdio_status {
	unsigned int mode;
	unsigned int phy_state;
	unsigned int phy_ticks;
	unsigned int phy_addr;
	unsigned int pending_status;
	unsigned int link_status;
	unsigned int macsel;
	unsigned int emac_txidver;
};

/*
 * MDIO events
 */
#define MDIO_EVENT_NOCHANGE      0   /* No change from previous status */
#define MDIO_EVENT_LINKDOWN      1   /* Link down event */
#define MDIO_EVENT_LINKUP        2   /* Link (or re-link) event */
#define MDIO_EVENT_PHYERROR      3   /* No PHY connected */

/*
 * MDIO link status values
 */
#define MDIO_LINKSTATUS_NOLINK   0
#define MDIO_LINKSTATUS_HD10     1
#define MDIO_LINKSTATUS_FD10     2
#define MDIO_LINKSTATUS_HD100    3
#define MDIO_LINKSTATUS_FD100    4
#define MDIO_LINKSTATUS_FD1000   5

/*
 * MDIO configuration mode flags
 */
#define MDIO_MODE_AUTONEG        0x0001 /* Use Autonegotiate         */
#define MDIO_MODE_HD10           0x0002 /* Use 10Mb/s Half Duplex    */
#define MDIO_MODE_FD10           0x0004 /* Use 10Mb/s Full Duplex    */
#define MDIO_MODE_HD100          0x0008 /* Use 100Mb/s Half Duplex   */
#define MDIO_MODE_FD100          0x0010 /* Use 100Mb/s Full Duplex   */
#define MDIO_MODE_FD1000         0x0020 /* Use 1000Mb/s Full Duplex  */
#define MDIO_MODE_LOOPBACK       0x0040 /* Use PHY Loopback          */
#define MDIO_MODE_NWAYACTIVE     0x0080 /* NWAY is currently active  */
#define MDIO_MODE_EXTLOOPBACK    0x0100 /* Use external PHY Loopback */

/*
 * MDIO states in the PHY state machine
 */
#define MDIO_PHY_MDIOINIT        0
#define MDIO_PHY_RESET           1
#define MDIO_PHY_NWAYSTART       2
#define MDIO_PHY_NWAYWAIT        3
#define MDIO_PHY_LINKWAIT        4
#define MDIO_PHY_LINKED          5

/*
 * PHY control registers
 */
#define MDIO_PHY_REG_CONTROL     0
#define MDIO_PHY_REG_STATUS      1
#define MDIO_PHY_REG_ID1         2
#define MDIO_PHY_REG_ID2         3
#define MDIO_PHY_REG_ADVERTISE   4
#define MDIO_PHY_REG_PARTNER     5
#define MDIO_PHY_REG_1000CONTROL 9
#define MDIO_PHY_REG_1000STATUS  0xA
#define MDIO_PHY_REG_EXTSTATUS   0xF
#define MDIO_PHY_REG_SHADOW      0x18
#define MDIO_PHY_REG_ACCESS      0x1C

#define MDIO_PHY_B_SPEEDMSB      (1<<6)
#define MDIO_PHY_B_DUPLEXFULL    (1<<8)
#define MDIO_PHY_B_AUTORESTART   (1<<9)
#define MDIO_PHY_B_ISOLATE       (1<<10)
#define MDIO_PHY_B_POWERDOWN     (1<<11)
#define MDIO_PHY_B_AUTONEGEN     (1<<12)
#define MDIO_PHY_B_SPEEDLSB      (1<<13)
#define MDIO_PHY_B_SPEED100      (1<<13)
#define MDIO_PHY_B_LOOPBACK      (1<<14)
#define MDIO_PHY_B_RESET         (1<<15)    /* for CONTROL */

#define MDIO_PHY_B_FD100         (1<<14)
#define MDIO_PHY_B_HD100         (1<<13)
#define MDIO_PHY_B_FD10          (1<<12)
#define MDIO_PHY_B_HD10          (1<<11)
#define MDIO_PHY_B_EXTSTATUS     (1<<8)
#define MDIO_PHY_B_NOPREAMBLE    (1<<6)
#define MDIO_PHY_B_AUTOCOMPLETE  (1<<5)
#define MDIO_PHY_B_REMOTEFAULT   (1<<4)
#define MDIO_PHY_B_AUTOCAPABLE   (1<<3)
#define MDIO_PHY_B_LINKSTATUS    (1<<2)
#define MDIO_PHY_B_JABBER        (1<<1)
#define MDIO_PHY_B_EXTENDED      (1<<0)     /* for STATUS */

#define MDIO_PHY_B_NEXTPAGE      (1<<15)
#define MDIO_PHY_B_ACK           (1<<14)
#define MDIO_PHY_B_FAULT         (1<<13)
#define MDIO_PHY_B_PAUSE         (1<<10)
#define MDIO_PHY_B_AFD100        (1<<8)
#define MDIO_PHY_B_AHD100        (1<<7)
#define MDIO_PHY_B_AFD10         (1<<6)
#define MDIO_PHY_B_AHD10         (1<<5)
#define MDIO_PHY_B_MSGMASK       (0x1F)
#define MDIO_PHY_B_MSG           (1<<0)     /* for ADVERTISE and PARTNER */

#define MDIO_PHY_ADV_FD1000      (1<<9)     /* for 1000CONTROL */

#define MDIO_PHY_PRT_FD1000      (1<<11)    /* for 1000STATUS */

#define MDIO_PHY_EXT_FD1000      (1<<13)    /* for EXTSTATUS */

#define MDIO_PHY_B_EXTLOOPBACK   0x8400
#define MDIO_PHY_B_RGMIIMODE     0xF080
#define MDIO_PHY_B_INBAND        0xF1C7     /* for SHADOW */

#define MDIO_PHY_B_COPPER        0xFC00     /* for ACCESS */


#define mdio_phy_read(regadr, phyadr)                   \
        mdio_set_reg(MDIO_USERACCESS0, MDIO_B_GO |      \
		     ((phyadr & 0x1f) << 16) |          \
		     ((regadr & 0x1f) << 21))

#define mdio_phy_write(regadr, phyadr, data)            \
        mdio_set_reg(MDIO_USERACCESS0, MDIO_B_GO |      \
                     MDIO_B_WRITE |                     \
		     ((phyadr & 0x1f) << 16) |          \
		     ((regadr & 0x1f) << 21) |          \
                     ((data & 0xffff)))

#define mdio_phy_wait()                                 \
        while(mdio_get_reg(MDIO_USERACCESS0) & MDIO_B_GO)

#define mdio_phy_wait_res(results)  do {                         \
            while(mdio_get_reg(MDIO_USERACCESS0) & MDIO_B_GO);  \
            results = mdio_get_reg(MDIO_USERACCESS0) & 0xffff;     \
        } while(0)

#define mdio_phy_wait_res_ack(results, ack)  do {                      \
            while(mdio_get_reg(MDIO_USERACCESS0) & MDIO_B_GO);        \
            results = mdio_get_reg(MDIO_USERACCESS0) & 0xffff;           \
            ack = (mdio_get_reg(MDIO_USERACCESS0) & MDIO_B_ACK) >> 29; \
        } while(0)

extern int mdio_init(unsigned int txid_version);
extern unsigned int mdio_get_status(void);
extern unsigned int mdio_timer_tick(void);
extern unsigned int mdio_get_macsel(void);

#endif /* __KERNEL__ */
#endif /* __ASM_C6X_MDIO_H_ */

