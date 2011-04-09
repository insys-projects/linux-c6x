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

#ifndef __ASM_C6X_CPSW_H_
#define __ASM_C6X_CPSW_H_

#define DEVICE_ETHERNET_SS_BASE		0x02090000
#define DEVICE_CPSW_BASE		0x02090800

struct port_info {
    u32 p_max_blks;
    u32 p_blk_cnt;
    u32 p_port_vlan;
    u32 p_tx_pri_map;
    u32 sl_sa_lo;
    u32 sl_sa_hi;
    u32 p_ts_ctl;
    u32 p_ts_seq_ltype;
    u32 p_ts_vlan_ltype;
    u8  rsvd2[12];
};

struct port_stats  {
    u32 rxgoodframes;
    u32 rxbroadcastframes;
    u32 rxmulticastframes;
    u32 rxpauseframes;
    u32 rxcrcerrors;
    u32 rxaligncodeerrors;
    u32 rxoversizedframes;
    u32 rxjabberframes;
    u32 rxundersizedframes;
    u32 rxfragments;
    u8  rsvd0[8];
    u32 rxoctets;
    u32 txgoodframes;
    u32 txbroadcastframes;
    u32 txmulticastframes;
    u32 txpauseframes;
    u32 txdeferredframes;
    u32 txcollisionframes;
    u32 txsinglecollframes;
    u32 txmultcollframes;
    u32 txexcessivecollisions;
    u32 txlatecollisions;
    u32 txunderrun;
    u32 txcarriersenseerrors;
    u32 txoctets;
    u32 octetframes64;
    u32 octetframes65t127;
    u32 octetframes128t255;
    u32 octetframes256t511;
    u32 octetframes512t1023;
    u32 octetframes1024tup;
    u32 netoctets;
    u32 rxsofoverruns;
    u32 rxmofoverruns;
    u32 rxdmaoverruns;
    u8  rsvd4[112];
};

struct cpsw_regs {
    u32 idver;
    u32 control;
    u32 emcontrol;
    u32 stat_port_en;
    u32 ptype;
    u8  rsvd0[8];
    u32 gap_thresh_cpgmac_sl;
    u32 tx_start_wds;
    u32 flow_control;
    u8  rsvd1[12];
    u32 p0_cppi_src_id;
    u32 p0_port_vlan;
    u32 p0_rx_pri_map;
    u32 p0_rx_maxlen;
    u8  rsvd3[28];
    struct port_info *p1;
    struct port_info *p2;
    u8  rsvd5[576];
    struct port_stats *s1;
    struct port_stats *s2;
    u8  rsvd6[256];
    u32 ale_id;
    u8  rsvd7[4];
    u32 ale_control;
    u8  rsvd8[4];
    u32 ale_prescale;
    u8  rsvd9[4];
    u32 ale_unknown_vlan;
    u8  rsvd10[4];
    u32 ale_table_control;
    u8  rsvd11[16];
    u32 ale_table_word2;
    u32 ale_table_word1;
    u32 ale_table_word0;
    u32 ale_port_control[3];
    u8  rsvd12[12];
}

struct cpgmac_sl {
    u32 idver;
    u32 maccontrol;
    u32 macstatus;
    u32 soft_reset;
    u32 rx_maxlen;
    u8  rsvd0[4];
    u32 rx_pause;
    u32 tx_pause;
    u32 emcontrol;
    u32 pri_map;
    u8  rsvd1[24];
};

/* Enable full duplex mode */
#define CPGMAC_SL_MACCONTROL_FULLDUPLEX_EN	(1 << 0)

/* Enable loopback mode */
#define	CPGMAC_SL_MACCONTROL_LOOPBACK_EN	(1 << 1)

/* Enable Rx flow control mode */
#define CPGMAC_SL_MACCONTROL_RX_FLOW_EN		(1 << 3)

/* Enable Tx flow control mode */
#define CPGMAC_SL_MACCONTROL_TX_FLOW_EN		(1 << 4)

/* Enable GMII */
#define CPGMAC_SL_MACCONTROL_GMII_EN		(1 << 5)

/* Enable Tx pacing */
#define CPGMAC_SL_MACCONTROL_TX_PACE_EN		(1 << 6)

/* Enable Gigabit mode */
#define CPGMAC_SL_MACCONTROL_GIG_EN		(1 << 7)

/* Enable Tx short gap */
#define	CPGMAC_SL_MACCONTROL_TX_SHORT_GAP_EN	(1 << 10)

/* Enable idle mode */
#define CPGMAC_SL_MACCONTROL_CMD_IDLE_EN	(1 << 11)

/* Set IFCTL_A bit to 1 */
#define CPGMAC_SL_MACCONTROL_IFCTL_A_EN		(1 << 15)

/* Set IFCTL_B bit to 1 */
#define CPGMAC_SL_MACCONTROL_IFCTL_B_EN		(1 << 16)

/* Enable forced Gigabit mode */
#define CPGMAC_SL_MACCONTROL_GIG_FORCE_EN	(1 << 17)

/* Enable external control mode */
#define CPGMAC_SL_MACCONTROL_EXT_EN		(1 << 18)

/* Enable Rx copy error frames mode */
#define CPGMAC_SL_MACCONTROL_RX_CEF_EN		(1 << 22)

/* Enable Rx copy short frames mode */
#define CPGMAC_SL_MACCONTROL_RX_CSF_EN		(1 << 23)

/* Enable Rx copy MAC control frames mode */
#define CPGMAC_SL_MACCONTROL_RX_CMF_EN		(1 << 24)

#define CPGMAC_SL_RX_MAXLEN_REG_RX_MAXLEN_MASK	(0x00003FFFu)

#define CPSW_3GF_SL_SA_LO_REG_MACSRCADDR_7_0_SHIFT	(0x00000008u)
#define CPSW_3GF_SL_SA_LO_REG_MACSRCADDR_15_8_SHIFT	(0x00000000u)
#define CPSW_3GF_SL_SA_HI_REG_MACSRCADDR_47_40_SHIFT	(0x00000000u)
#define CPSW_3GF_SL_SA_HI_REG_MACSRCADDR_39_32_SHIFT	(0x00000008u)
#define CPSW_3GF_SL_SA_HI_REG_MACSRCADDR_31_24_SHIFT	(0x00000010u)
#define CPSW_3GF_SL_SA_HI_REG_MACSRCADDR_23_16_SHIFT	(0x00000018u)

#define CPSW_3GF_CPSW_CONTROL_REG_P0_ENABLE	(0x00000004u)
#define CPSW_3GF_CPSW_CONTROL_REG_VLAN_AWARE	(0x00000002u)

#define CPSW_3GF_STAT_PORT_EN_REG_P0A_STAT_EN	(0x00000001u)
#define CPSW_3GF_STAT_PORT_EN_REG_P0B_STAT_EN	(0x00000002u)
#define CPSW_3GF_STAT_PORT_EN_REG_P1_STAT_EN	(0x00000004u)
#define CPSW_3GF_STAT_PORT_EN_REG_P2_STAT_EN	(0x00000008u)

#define CPSW_3GF_ALE_CONTROL_REG_ENABLE_ALE	(0x80000000u)
#define CPSW_3GF_ALE_CONTROL_REG_CLEAR_TABLE	(0x40000000u)
#define CPSW_3GF_ALE_CONTROL_REG_ALE_VLAN_AWARE	(0x00000004u)
#define CPSW_3GF_ALE_CONTROL_REG_RATE_LIMIT_TX	(0x00000008u)
	
#define CPSW_3GF_ALE_PORTSTATE_DISABLED 	0
#define CPSW_3GF_ALE_PORTSTATE_BLOCKED		1
#define CPSW_3GF_ALE_PORTSTATE_LEARN		2
#define CPSW_3GF_ALE_PORTSTATE_FORWARD		3

#define CPSW_3GF_ALE_PORT_CONTROL_REG_PORT_STATE_MASK	(0x00000003u)
#define CPSW_3GF_ALE_PORT_CONTROL_REG_DROP_UNTAGGED	(0x00000004u)
#define CPSW_3GF_ALE_PORT_CONTROL_REG_VID_INGRESS_CHECK	(0x00000008u)
#define CPSW_3GF_ALE_PORT_CONTROL_REG_NO_LEARN		(0x00000010u)
#define CPSW_3GF_ALE_PORT_CONTROL_REG_MCAST_LIMIT_MASK	(0x00FF0000u)
#define CPSW_3GF_ALE_PORT_CONTROL_REG_MCAST_LIMIT_SHIFT	(0x00000010u)
#define CPSW_3GF_ALE_PORT_CONTROL_REG_BCAST_LIMIT_MASK	(0xFF000000u)
#define CPSW_3GF_ALE_PORT_CONTROL_REG_BCAST_LIMIT_SHIFT	(0x00000018u)

#define CPSW_3GF_ALE_UNKNOWN_VLAN_MEMBER_LIST_MASK	(0x0000003Fu)
#define CPSW_3GF_ALE_UNKNOWN_VLAN_MEMBER_LIST_SHIFT	(0x00000000u)

#define CPSW_3GF_ALE_UNKNOWN_VLAN_MCAST_FLOOD_MASK_MASK	(0x00003F00u)
#define CPSW_3GF_ALE_UNKNOWN_VLAN_MCAST_FLOOD_MASK_SHIFT (0x00000008u)

#define CPSW_3GF_ALE_UNKNOWN_VLAN_MCAST_FLOOD_MASK_MASK (0x003F0000u)
#define CPSW_3GF_ALE_UNKNOWN_VLAN_MCAST_FLOOD_MASK_SHIFT (0x00000010u)

#define CPSW_3GF_ALE_UNKNOWN_VLAN_FORCE_UNTAGGED_EGRESS_MASK (0x3F000000u)
#define CPSW_3GF_ALE_UNKNOWN_VLAN_FORCE_UNTAGGED_EGRESS_SHIFT (0x00000018u)

#define cpsw_setbit_reg(reg, val) \
        *((volatile u32 *) (reg)) |= (u32) (val)
	    
#define cpsw_clearbit_reg(reg, val) \
        *((volatile u32 *) (reg)) &= ~((u32) (val))
        
#define cpsw_set_reg(reg, val) \
        *((volatile u32 *) (reg)) = (u32) (val)
        
#define cpsw_get_reg(reg) \
        *((volatile u32 *) (reg))

#define cpsw_addr_reg(reg) \
        ((volatile u32 *) (reg))

#endif /* __ASM_C6X_CPSW_H_ */

