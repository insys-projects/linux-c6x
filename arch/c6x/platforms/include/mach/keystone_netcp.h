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

#ifndef __MACH_C6X_KEYSTONE_NETCP_H
#define __MACH_C6X_KEYSTONE_NETCP_H

#include <linux/netdevice.h>

#define MAX_SIZE_STREAM_BUFFER		        1520

#define DEVICE_PA_CDMA_GLOBAL_CFG_BASE		0x02004000
#define DEVICE_PA_CDMA_TX_CHAN_CFG_BASE		0x02004400
#define DEVICE_PA_CDMA_RX_CHAN_CFG_BASE		0x02004800
#define DEVICE_PA_CDMA_RX_FLOW_CFG_BASE		0x02005000

#define DEVICE_PA_CDMA_RX_NUM_CHANNELS		24
#define DEVICE_PA_CDMA_RX_NUM_FLOWS		32
#define DEVICE_PA_CDMA_TX_NUM_CHANNELS		9

#define DEVICE_EMACSL_BASE(x)			(0x02090900 + (x)*0x040)
#define DEVICE_N_GMACSL_PORTS			2
#define DEVICE_EMACSL_RESET_POLL_COUNT		100

#define DEVICE_RX_CDMA_TIMEOUT_COUNT		1000

#define DEVICE_RX_INT_THRESHOLD                 3
#define DEVICE_TX_INT_THRESHOLD                 3

#define DEVICE_PSTREAM_CFG_REG_ADDR             0x02000604
#define DEVICE_PSTREAM_CFG_REG_VAL_ROUTE_PDSP0	0

struct pdsp_platform_data {
	unsigned int pdsp;
	char        *firmware;
	int          firmware_version;
};

struct netcp_platform_data {

	/* Rx/tx interrupts */
	unsigned int rx_irq;
	unsigned int tx_irq;

	/* PA PDSP */
	struct pdsp_platform_data pa_pdsp;

	/* QM PDSP */
	struct pdsp_platform_data qm_pdsp;
};

#if defined(CONFIG_SOC_TMS320C6670) || defined(CONFIG_SOC_TMS320C6678)
#define EMAC_ARCH_HAS_INTERRUPT
#define EMAC_ARCH_HAS_MAC_ADDR
#define EFUSE_REG_MAC_ADDR	        0x2620110
#define emac_arch_get_mac_addr  	emac_arch_get_mac_addr_from_efuse

/* Read the e-fuse value as 32 bit values to be endian independent */
static int inline emac_arch_get_mac_addr_from_efuse(char *x)
{
	unsigned int addr0, addr1;

	addr1 = __raw_readl(EFUSE_REG_MAC_ADDR + 4);
	addr0 = __raw_readl(EFUSE_REG_MAC_ADDR);

	x[0] = (addr1 & 0x0000ff00) >> 8;
	x[1] = addr1 & 0x000000ff;
	x[2] = (addr0 & 0xff000000) >> 24;
	x[3] = (addr0 & 0x00ff0000) >> 16;
	x[4] = (addr0 & 0x0000ff00) >> 8;
	x[5] = addr0 & 0x000000ff;

	return 0;
}
#endif

/*
 * Firmware 
 */
#include <asm/byteorder.h>
#define DEVICE_PA_PDSP_FIRMWARE "PA_PDSP_DEFAULT"
#ifdef CONFIG_CPU_BIG_ENDIAN
#define DEVICE_QM_PDSP_FIRMWARE "QMSS_PDSP_ACC48_BE"
#else
#define DEVICE_QM_PDSP_FIRMWARE "QMSS_PDSP_ACC48_LE"
#endif

/*
 * Configure the streaming switch
 */
static inline void streaming_switch_setup(void)
{
	__raw_writel(DEVICE_PSTREAM_CFG_REG_VAL_ROUTE_PDSP0,
		     DEVICE_PSTREAM_CFG_REG_ADDR);
}

/* Register offsets */
#define CPGMACSL_REG_ID		                0x00
#define CPGMACSL_REG_CTL	                0x04
#define CPGMACSL_REG_STATUS	                0x08
#define CPGMACSL_REG_RESET	                0x0c
#define CPGMACSL_REG_MAXLEN	                0x10
#define CPGMACSL_REG_BOFF	                0x14
#define CPGMACSL_REG_RX_PAUSE	                0x18
#define CPGMACSL_REG_TX_PAURSE	                0x1c
#define CPGMACSL_REG_EM_CTL	                0x20
#define CPGMACSL_REG_PRI	                0x24

/* Soft reset register values */
#define CPGMAC_REG_RESET_VAL_RESET_MASK		(1 << 0)
#define CPGMAC_REG_RESET_VAL_RESET		(1 << 0)

/* Maxlen register values */
#define CPGMAC_REG_MAXLEN_LEN			0x3fff

#define GMACSL_RX_ENABLE_RCV_CONTROL_FRAMES	(1 << 24)
#define GMACSL_RX_ENABLE_RCV_SHORT_FRAMES	(1 << 23)
#define GMACSL_RX_ENABLE_RCV_ERROR_FRAMES	(1 << 22)
#define GMACSL_RX_ENABLE_EXT_CTL		(1 << 18)
#define GMACSL_RX_ENABLE_GIG_FORCE		(1 << 17)
#define GMACSL_RX_ENABLE_IFCTL_B		(1 << 16)
#define GMACSL_RX_ENABLE_IFCTL_A		(1 << 15)
#define GMACSL_RX_ENABLE_CMD_IDLE		(1 << 11)
#define GMACSL_TX_ENABLE_SHORT_GAP		(1 << 10)
#define GMACSL_ENABLE_GIG_MODE			(1 <<  7)
#define GMACSL_TX_ENABLE_PACE			(1 <<  6)
#define GMACSL_ENABLE				(1 <<  5)
#define GMACSL_TX_ENABLE_FLOW_CTL		(1 <<  4)
#define GMACSL_RX_ENABLE_FLOW_CTL		(1 <<  3)
#define GMACSL_ENABLE_LOOPBACK			(1 <<  1)
#define GMACSL_ENABLE_FULL_DUPLEX		(1 <<  0)

#define GMACSL_RET_OK				0
#define GMACSL_RET_INVALID_PORT			-1
#define GMACSL_RET_WARN_RESET_INCOMPLETE	-2
#define GMACSL_RET_WARN_MAXLEN_TOO_BIG		-3
#define GMACSL_RET_CONFIG_FAIL_RESET_ACTIVE	-4

/*
 * MAC Configuration information
 */
struct emac_config {
	u32 flags;
	u8  enetaddr[6];
};

struct mac_sliver {
    u32 max_rx_len;	/* Maximum receive packet length */
    u32 ctl;		/* Control bitfield */
};

#endif /* __MACH_C6X_KEYSTONE_NETCP_H */

