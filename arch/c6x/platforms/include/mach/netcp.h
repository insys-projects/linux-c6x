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

#ifndef __MACH_C6X_NETCP_H
#define __MACH_C6X_NETCP_H

#define BOOTBITMASK(x,y)	(((((u32)1 << (((u32)x)-((u32)y)+(u32)1)) \
				- (u32)1 ))   <<  ((u32)y))
#define BOOT_READ_BITFIELD(z,x,y)	(((u32)z) & BOOTBITMASK(x,y)) >> (y)
#define BOOT_SET_BITFIELD(z,f,x,y)	(((u32)z) & ~BOOTBITMASK(x,y)) | \
					((((u32)f) << (y)) & BOOTBITMASK(x,y))

/*
 * SGMII registers
 */
#define SGMII_BASE_ADDRESS	{ 0x02090100, 0x02090200 }
#define SGMII_IDVER_REG(x)	(0x02090100 + (x * 0x100) + 0x000)
#define SGMII_SRESET_REG(x)	(0x02090100 + (x * 0x100) + 0x004)
#define SGMII_CTL_REG(x)	(0x02090100 + (x * 0x100) + 0x010)
#define SGMII_STATUS_REG(x)	(0x02090100 + (x * 0x100) + 0x014)
#define SGMII_MRADV_REG(x)	(0x02090100 + (x * 0x100) + 0x018)
#define SGMII_LPADV_REG(x)	(0x02090100 + (x * 0x100) + 0x020)
#define SGMII_TXCFG_REG(x)	(0x02090100 + (x * 0x100) + 0x030)
#define SGMII_RXCFG_REG(x)	(0x02090100 + (x * 0x100) + 0x034)
#define SGMII_AUXCFG_REG(x)	(0x02090100 + (x * 0x100) + 0x038)

#define SGMII_REG_STATUS_FIELD_LOCK	(1<<4)

#define CPSW_CTL_P2_PASS_PRI_TAGGED	(1 << 5)
#define CPSW_CTL_P1_PASS_PRI_TAGGED	(1 << 4)
#define CPSW_CTL_P0_PASS_PRI_TAGGED	(1 << 3)
#define CPSW_CTL_P0_ENABLE		(1 << 2)
#define CPSW_CTL_VLAN_AWARE		(1 << 1)
#define CPSW_CTL_FIFO_LOOPBACK		(1 << 0)

#define DEVICE_CPSW_BASE		(0x02090800)

/* Register offsets */
#define CPSW_REG_CTL			0x004
#define CPSW_REG_STAT_PORT_EN		0x00c
#define CPSW_REG_MAXLEN			0x040
#define CPSW_REG_ALE_CONTROL		0x608
#define CPSW_REG_ALE_PORTCTL(x)		(0x640 + (x)*4)

/* Register values */
#define CPSW_REG_VAL_STAT_ENABLE_ALL		0xf
#define CPSW_REG_VAL_ALE_CTL_RESET_AND_ENABLE	((u32)0xc0000000)
#define CPSW_REG_VAL_PORTCTL_FORWARD_MODE	0x3

#define DEVICE_CPSW_NUM_PORTS		3
#define MAX_SIZE_STREAM_BUFFER		1520

struct keystone_platform_data {

};

/* API */
int hw_cpsw_config(u32 ctl, u32 max_pkt_size);

#endif /* __MACH_C6X_NETCP_H */

