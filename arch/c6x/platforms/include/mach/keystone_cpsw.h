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
#ifndef __MACH_C6X_KEYSTONE_CPSW_H
#define __MACH_C6X_KEYSTONE_CPSW_H

#define ETHERNET_MTU	                        VLAN_ETH_FRAME_LEN

#define CPSW_NUM_PORTS		                3
#define CPSW_CPPI_PORT_NUM	                0
#define CPSW_SGMII0_PORT_NUM                    1
#define CPSW_SGMII1_PORT_NUM                    2
#define CPSW_NUM_STATS_MODULES                  2
#define CPSW_STATSA_MODULE                      0 /* For switch port 0 */
#define CPSW_STATSB_MODULE                      1 /* For switch port 1 and 2 */

#define CPSW_ALE_AGEOUT                         10
#define CPSW_ALE_ENTRIES                        1024
#define CPSW_ALE_PORTS                          2

/*
 * CPSW registers
 */
#define DEVICE_CPSW_BASE		        (0x02090800)
#define DEVICE_CPSW_REGION_SIZE			0x800

/* Register offsets */
#define CPSW_REG_CTL			        0x004
#define CPSW_REG_STAT_PORT_EN		        0x00c
#define CPSW_REG_FLOW_CTL		        0x024
#define CPSW_REG_P0_MAXLEN		        0x040
#define CPSW_REG_MAC_SA_LO(s)                   (0x70 + (s)*0x30)
#define CPSW_REG_MAC_SA_HI(s)                   (0x74 + (s)*0x30)
#define CPSW_REG_STATS(x)                       (0x300 + (x)*0x100)
#define CPSW_REG_ALE_BASE		        0x600

/* Register values */
#define CPSW_CTL_P2_PASS_PRI_TAGGED	        (1 << 5)
#define CPSW_CTL_P1_PASS_PRI_TAGGED	        (1 << 4)
#define CPSW_CTL_P0_PASS_PRI_TAGGED	        (1 << 3)
#define CPSW_CTL_P0_ENABLE		        (1 << 2)
#define CPSW_CTL_VLAN_AWARE		        (1 << 1)
#define CPSW_CTL_FIFO_LOOPBACK		        (1 << 0)

#define CPSW_REG_VAL_STAT_ENABLE_ALL		0xf

#define CPSW_MASK_ALL_PORTS			7
#define CPSW_MASK_PHYS_PORTS			6
#define CPSW_MASK_NO_PORTS			0

#define CPSW_NON_VLAN_ADDR			-1

struct keystone_cpsw_stats {
	u32 rx_good_frames;
	u32 rx_broadcast_frames;
	u32 rx_multicast_frames;
	u32 rx_pause_frames;
	u32 rx_crc_errors;
	u32 rx_align_code_errors;
	u32 rx_oversized_frames;
	u32 rx_jabber_frames;
	u32 rx_undersized_frames;
	u32 rx_fragments;
	u32 reserved[2];
	u32 rx_bytes;
	u32 tx_good_frames;
	u32 tx_broadcast_frames;
	u32 tx_multicast_frames;
	u32 tx_pause_frames;
	u32 tx_deferred_frames;
	u32 tx_collision_frames;
	u32 tx_single_coll_frames;
	u32 tx_mult_coll_frames;
	u32 tx_excessive_collisions;
	u32 tx_late_collisions;
	u32 tx_underrun;
	u32 tx_carrier_senser_errors;
	u32 tx_bytes;
	u32 tx_64byte_frames;
	u32 tx_65_to_127byte_frames;
	u32 tx_128_to_255byte_frames;
	u32 tx_256_to_511byte_frames;
	u32 tx_512_to_1023byte_frames;
	u32 tx_1024byte_frames;
	u32 net_bytes;
	u32 rx_sof_overruns;
	u32 rx_mof_overruns;
	u32 rx_dma_overruns;
};

/*
 * CPSW ports are 0 for host port, 1 for SGMII0 and 2 for SGMII1
 *
 *
 *                     +-----------------------+
 *                     |                       |
 *                     |         CPSW          |
 *                     |    3 port Ethernet    |
 *                     |        switch         |
 *                     |                       |
 *                     |                     1 |<----> SGMII0 port 
 *                     |                       |
 *  Host port <------->| 0                     |
 *  (to NetCP)         |                       |
 *                     |                     2 |<----> SGMII1 port
 *                     |                       |
 *                     |                       |
 *                     |                       |
 *                     +-----------------------+
 *
 */
#define cpsw_get_slave(cpsw_port) ((cpsw_port) - 1)
#define cpsw_get_port(sgmii_port) ((sgmii_port) + 1)

static inline struct keystone_cpsw_stats* cpsw_get_stats(int module)
{
	if ((module < 0) || (module >= CPSW_NUM_STATS_MODULES))
		return NULL;

	return (struct keystone_cpsw_stats*)
		(DEVICE_CPSW_BASE + CPSW_REG_STATS(module));
}

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

#endif /* __MACH_C6X_KEYSTONE_CPSW_H */
