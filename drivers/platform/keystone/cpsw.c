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
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/keystone/cpsw.h>

#include <asm/setup.h>
#include <asm/machdep.h>
#include <asm/io.h>

#include <mach/keystone_cpsw.h>


static int find_ale_next_free_entry(void)
{
	struct ale_regs *ale = (struct ale_regs *)(KEYSTONE_CPSW_BASE +
						   0x600);
	unsigned int tmp, i;

	/* 
	 * Search the ALE table for the next available free ALE entry
	 * Break if we find a free entry
	 */
	for (i = 0; i < CPSW_3GF_NUM_ALE_ENTRIES; i++) {
		tmp = __raw_readl(&ale->ale_table_control);
		tmp &= ~(CPSW_3GF_ALE_TABLE_CONTROL_ENTRY_POINTER_MASK |
			CPSW_3GF_ALE_TABLE_CONTROL_WRITE_RDZ_MASK);
		tmp |= (0 << CPSW_3GF_ALE_TABLE_CONTROL_WRITE_RDZ_SHIFT);
		tmp |= i;
		__raw_writel(tmp, &ale->ale_table_control);

		/* Read Table Word 1 to check entry type */
		tmp = __raw_readl(&ale->ale_table_word1);
		tmp &= CPSW_3GF_ALE_TABLE_WORD1_ENTRY_TYPE_MASK;
		tmp >>= 28;
		
		if (tmp == ALE_ENTRYTYPE_FREE)
			break;
	}

	/* No free ALE entry found. return error. */
	if (i == CPSW_3GF_NUM_ALE_ENTRIES)
		return -1;

	/* Found a free ALE entry at index "i" */
	return i;
}

/* 
 * This API adds a unicast ALE entry to the switch ALE table
 * 
 * por_tnum		Switch port number.
 *
 * mac_address		MAC address to configure on the switch.
 * 
 * blocked		Flag to indicate packet matching this entry must be
 *                      blocked/dropped
 *                      
 * secure		Flag to indicate if the packet with matching source
 *                      address should be dropped if the received port is
 *                      not same as table entry.
 */
static int add_unicast_ale_entry (u32 port_num, u8 mac_address[6],
				  u32 blocked, u32 secure)
{
	struct ale_regs *ale = (struct ale_regs *)(KEYSTONE_CPSW_BASE +
					0x600);
	unsigned int tmp;
	int ale_tbl_index;

	// TODO: No update ALE address supported yet. Only permanent addresses

	/* Program the ALE with the MAC address.
	 *
	 * The ALE entries determine the switch port to which any
	 * matching received packet must be forwarded to.
	 */
	if ((ale_tbl_index = find_ale_next_free_entry()) < 0)           
		return -1;
	else {
		/* Found a free ALE entry to program our MAC address */
		tmp = (mac_address[5] |  mac_address[4] << 8 |
			mac_address[3] << 16 |	 mac_address[2] << 24);
		__raw_writel(tmp, &ale->ale_table_word0);

		tmp = (mac_address[1] |  mac_address[0] << 8);
		tmp |=  ((ALE_UCASTTYPE_UCAST_NOAGE << 30) |
			(1 << 28));
		__raw_writel(tmp, &ale->ale_table_word1);

		tmp = ((secure << 0) | (blocked << 1) | (port_num << 2));
		__raw_writel(tmp, &ale->ale_table_word2);

		tmp = __raw_readl(&ale->ale_table_control);
		tmp &= ~(CPSW_3GF_ALE_TABLE_CONTROL_ENTRY_POINTER_MASK |
			CPSW_3GF_ALE_TABLE_CONTROL_WRITE_RDZ_MASK);
		tmp |= (1 << CPSW_3GF_ALE_TABLE_CONTROL_WRITE_RDZ_SHIFT);
		tmp |= ale_tbl_index;
		__raw_writel(tmp, &ale->ale_table_control);
	}

	/* Done with adding unicast entry address */
	return 0;
}

/*
 * This API adds a multicast ALE entry to the switch ALE table
 *  
 * port_mask		MAC ports that the packets destined to matching 
 *                      multicast address must be forwarded to.
 *
 * mac_address		Multicast address.
 *   
 * super		Flag to indicate if this is a supervisory packet
 *
 * fwd_state		The state in which matching multicast packets will be
 *                      forwarded.
 */
static int add_multicast_ale_entry(u32 port_mask, u8 mac_address[6],
				   u32 super, u32 fwd_state)
{
	struct ale_regs *ale = (struct ale_regs *)(KEYSTONE_CPSW_BASE +
					0x600);
	unsigned int tmp;
	int ale_tbl_index;

	//TODO:  No update ALE address supported yet. Only permanent addresses

	/* Program the ALE with the MAC address.
	 *
	 * The ALE entries determine the switch port to which any
	 * matching received packet must be forwarded to.
	 */
	if ((ale_tbl_index = find_ale_next_free_entry()) < 0)           
		return -1;
	else {
		/* Found a free ALE entry to program our MAC address */
		tmp =  (mac_address[5] |  mac_address[4] << 8 |
			mac_address[3] << 16 |	 mac_address[2] << 24);
		__raw_writel(tmp, &ale->ale_table_word0);

		tmp = (mac_address[1] |  mac_address[0] << 8);
		tmp |=  ((fwd_state << 30) | (1 << 28));
		__raw_writel(tmp, &ale->ale_table_word1);

		tmp = ((super << 1) | (port_mask << 2));
		__raw_writel(tmp, &ale->ale_table_word2);
	
		tmp = __raw_readl(&ale->ale_table_control);
		tmp &= ~(CPSW_3GF_ALE_TABLE_CONTROL_ENTRY_POINTER_MASK |
			CPSW_3GF_ALE_TABLE_CONTROL_WRITE_RDZ_MASK);
		tmp |= (1 << CPSW_3GF_ALE_TABLE_CONTROL_WRITE_RDZ_SHIFT);
		tmp |= ale_tbl_index;
		__raw_writel(tmp, &ale->ale_table_control);
	}

	/* Done with adding the multicast entry address */
	return 0;
}

static unsigned int init_mac(unsigned int macportnum, u8* mac_address,
			     unsigned int mtu)
{

	struct cpgmac_sl *mac_sl = (struct cpgmac_sl *)(KEYSTONE_CPSW_BASE +
					0x100 + 0x40 * macportnum);
	struct port_info *p_info = (struct port_info *)(KEYSTONE_CPSW_BASE +
					0x60 + 0x30 * macportnum);
	
	unsigned int tmp;

	/*
	 * Reset the MAC sliver
	 * Poll till the reset has occurred
	 */
	__raw_writel(0x00000001, &mac_sl->soft_reset);
	while(__raw_readl(&mac_sl->soft_reset));           

	/*
	 * Setup the MAC Control Register for this port:
	 * (1) Enable Full duplex
	 * (2) Enable GMII
	 * (3) Enable Gigabit
	 * (4) Enable External Configuration. This enables
	 *	the "Full duplex" and "Gigabit" settings to be
	 *	controlled externally from SGMII
	 * (5) Don't enable any control/error/short frames
	 */
	tmp = __raw_readl(&mac_sl->maccontrol);
	tmp |= (CPGMAC_SL_MACCONTROL_FULLDUPLEX_EN |
		CPGMAC_SL_MACCONTROL_GMII_EN |
		CPGMAC_SL_MACCONTROL_GIG_EN |
		CPGMAC_SL_MACCONTROL_GIG_FORCE_EN);	
	__raw_writel(tmp, &mac_sl->maccontrol);
		
	/* Enable Short Frames */
	tmp = __raw_readl(&mac_sl->maccontrol);
	tmp |= CPGMAC_SL_MACCONTROL_RX_CSF_EN;
	__raw_writel(tmp, &mac_sl->maccontrol);

	/* Configure the MAC address for appropriate port */
	__raw_writel(((mac_address[0] << CPSW_3GF_SL_SA_LO_REG_MACSRCADDR_7_0_SHIFT) |
			(mac_address[1] << CPSW_3GF_SL_SA_LO_REG_MACSRCADDR_15_8_SHIFT)),
			&p_info->sl_sa_lo);
	__raw_writel(((mac_address[2] << CPSW_3GF_SL_SA_HI_REG_MACSRCADDR_23_16_SHIFT) |
			(mac_address[3] << CPSW_3GF_SL_SA_HI_REG_MACSRCADDR_31_24_SHIFT) |
			(mac_address[4] << CPSW_3GF_SL_SA_HI_REG_MACSRCADDR_39_32_SHIFT) |
			(mac_address[5] << CPSW_3GF_SL_SA_HI_REG_MACSRCADDR_47_40_SHIFT)),
			&p_info->sl_sa_hi);
 	
	/*
	 * Configure VLAN ID/CFI/Priority for appropriate port
	 * For now, we are not using VLANs so just configure them
	 * to all zeros
	 */
	__raw_writel(0, &p_info->p_port_vlan);

	/* 
	 * Configure the Receive Maximum length on this port,
	 * i.e., the maximum size the port can receive without
	 * any errors.
	 *
	 * Set the Rx Max length to the MTU configured for the
	 * interface.
	 */
	tmp = __raw_readl(&mac_sl->rx_maxlen);
	tmp &= ~CPGMAC_SL_RX_MAXLEN_REG_RX_MAXLEN_MASK;
	tmp |= mtu;
	__raw_writel(tmp, &mac_sl->rx_maxlen); 

	/* Done setting up the MAC port */
	return 0;
}

static void init_mdio (unsigned int macportnum)
{
	/* PHYs already enabled. Do nothing. Return success. */
}

static void init_switch(unsigned int mtu)
{

	struct cpsw_regs *cpsw = (struct cpsw_regs *)(KEYSTONE_CPSW_BASE);
	struct ale_regs *ale = (struct ale_regs *)(KEYSTONE_CPSW_BASE +
					0x600);
	unsigned int tmp;

	/*
	 * Enable the CPPI port, i.e., port 0 that does all
	 * the data streaming in/out of EMAC.
	 */
	tmp = __raw_readl(&cpsw->control);
	tmp |= CPSW_3GF_CPSW_CONTROL_REG_P0_ENABLE;
	tmp &= ~CPSW_3GF_CPSW_CONTROL_REG_VLAN_AWARE;
	__raw_writel(tmp, &cpsw->control);

	/* Write 0's to P0_PORT_VLAN register */
	__raw_writel(0, &cpsw->p0_port_vlan);

		
	/* Write mtu to the P0 Maxlen register */
	__raw_writel(mtu, &cpsw->p0_rx_maxlen);

	/*
	 * Enable statistics on both the port groups:
	 *
	 * MAC Sliver ports -   Port 1, Port 2
	 * CPPI Port        -   Port 0
	 */
	__raw_writel(CPSW_3GF_STAT_PORT_EN_REG_P0A_STAT_EN |
			CPSW_3GF_STAT_PORT_EN_REG_P0B_STAT_EN |
			CPSW_3GF_STAT_PORT_EN_REG_P1_STAT_EN |
			CPSW_3GF_STAT_PORT_EN_REG_P2_STAT_EN,
			&cpsw->stat_port_en);

	/*
	 * Setup the Address Lookup Engine (ALE) Configuration:
	 *	(1) Enable ALE.
	 *	(2) Clear stale ALE entries.
	 *	(3) Disable VLAN Aware lookups in ALE since
	 *	    we are not using VLANs by default.
	 *	(4) No Flow control
	 *	(5) Configure the Unknown VLAN processing 
	 *          properties for the switch, i.e., which 
	 *	    ports to send the packets to.
	 */
	tmp = __raw_readl(&ale->ale_control);
	tmp |= CPSW_3GF_ALE_CONTROL_REG_ENABLE_ALE |
		CPSW_3GF_ALE_CONTROL_REG_CLEAR_TABLE;
	tmp &= ~(CPSW_3GF_ALE_CONTROL_REG_ALE_VLAN_AWARE |
		CPSW_3GF_ALE_CONTROL_REG_RATE_LIMIT_TX);
	__raw_writel(tmp, &ale->ale_control);

	
	__raw_writel(125000000u/1000u, &ale->ale_prescale);

	tmp = __raw_readl(&ale->ale_unknown_vlan);
	tmp &= ~(CPSW_3GF_ALE_UNKNOWN_VLAN_MEMBER_LIST_MASK |
		CPSW_3GF_ALE_UNKNOWN_VLAN_MCAST_FLOOD_MASK_MASK |
		CPSW_3GF_ALE_UNKNOWN_VLAN_REG_MCAST_FLOOD_MASK_MASK |
		CPSW_3GF_ALE_UNKNOWN_VLAN_FORCE_UNTAGGED_EGRESS_MASK);
	tmp |= (7 << CPSW_3GF_ALE_UNKNOWN_VLAN_MEMBER_LIST_SHIFT) |
		(3 << CPSW_3GF_ALE_UNKNOWN_VLAN_MCAST_FLOOD_MASK_SHIFT) |
		(3 << CPSW_3GF_ALE_UNKNOWN_VLAN_REG_MCAST_FLOOD_MASK_SHIFT) |
		(7 << CPSW_3GF_ALE_UNKNOWN_VLAN_FORCE_UNTAGGED_EGRESS_SHIFT); 
	__raw_writel(tmp, &ale->ale_unknown_vlan);

	/* Done with switch configuration */
}

static void setup_ale_portconfig (unsigned int portnum)
{
	struct ale_regs *ale = (struct ale_regs *)(KEYSTONE_CPSW_BASE +
					0x600);
	unsigned int tmp;

	/* Configure the address in "Learning"/"Forward" state */
	tmp = __raw_readl(&ale->ale_port_control[portnum]);
	tmp |= (CPSW_3GF_ALE_PORTSTATE_FORWARD & 
		CPSW_3GF_ALE_PORT_CONTROL_REG_PORT_STATE_MASK);
	tmp &= ~(CPSW_3GF_ALE_PORT_CONTROL_REG_DROP_UNTAGGED |
		CPSW_3GF_ALE_PORT_CONTROL_REG_VID_INGRESS_CHECK);

	/*
	 * In dual emac mode, the only "Intelligent" port should be
	 * the Host/CPU/DMA port, i.e., Port 2.
	 *
	 * The ALE table is pre-populated with MAC unicast addresses
	 * corresponding to ports 1, 2 and no learning is required
	 * on these ports. Source Address Learning should be thus
	 * disabled on the GMAC ports, i.e.,Ports 1 and 2.
	 * Learning should be enabled only on Port 0, i.e., CPPI Port
	 */
	if (portnum != CPPI_PORT_NUM)
		tmp |= CPSW_3GF_ALE_PORT_CONTROL_REG_NO_LEARN;
	else
		tmp &= ~CPSW_3GF_ALE_PORT_CONTROL_REG_NO_LEARN;

	tmp &= ~(CPSW_3GF_ALE_PORT_CONTROL_REG_MCAST_LIMIT_MASK |
		CPSW_3GF_ALE_PORT_CONTROL_REG_BCAST_LIMIT_MASK);
	tmp |= (0 << CPSW_3GF_ALE_PORT_CONTROL_REG_MCAST_LIMIT_SHIFT) |
		(0 << CPSW_3GF_ALE_PORT_CONTROL_REG_BCAST_LIMIT_SHIFT);

	__raw_writel(tmp, &ale->ale_port_control[portnum]);

}

int cpsw_init (u8 macaddress[2][6])
{
	unsigned int	macportnum;
	unsigned int	portnum;

	/*
	 * Initialize the Sliver submodules for the
	 * two corresponding MAC ports.
	 */
	for (macportnum = 0; macportnum < 2; macportnum++)
	{
		/*
		 * In dual EMAC mode, each of MAC ports has a different MAC
		 * address. In switch mode, both MAC ports have the same
		 * MAC address as the switch
		 */
		init_mac(macportnum, &macaddress[macportnum][0], ETHERNET_MTU);

		/* Setup the PHYs by initializing the MDIO */
		init_mdio(macportnum);
	}

	/* Setup the Ethernet switch */
	init_switch(ETHERNET_MTU);

	/* Setup ALE Port configuration for all ports */
	for (portnum = 0; portnum < 2; portnum++)
		setup_ale_portconfig(portnum);

	add_unicast_ale_entry (0, macaddress[0], 0, 1);
	add_unicast_ale_entry (0, macaddress[1], 1, 1);

	return 0;
}


