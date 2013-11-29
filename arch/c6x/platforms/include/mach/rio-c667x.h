/*
 *  linux/arch/c6x/platforms/include/mach/rio-c667x.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2011, 2012 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef _MACH_RIO_C667X_H
#define _MACH_RIO_C667X_H

#ifdef __KERNEL__
#include <asm/setup.h>
#include <asm/cache.h>
#include <asm/hardware.h>
#include <asm/virtconvert.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/board.h>

#define KEYSTONE_RIO_GET_DID(x, m)	  (m ? (x & 0xffff) : ((x & 0x00ff0000) >> 16))
#define KEYSTONE_RIO_SET_DID(x, m)	  (m ? (x & 0xffff) : ((x & 0x000000ff) << 16))

#define DEVICE_REG32_W(x, y)               *(volatile unsigned int *)(x) = (y)
#define DEVICE_REG32_R(x)                 (*(volatile unsigned int *)(x))

#ifdef CONFIG_TMS320C6X_CACHES_ON
#define keystone_rio_data_sync_write(start, end) L2_cache_block_writeback(start, end);
#define keystone_rio_data_sync_read(start, end)  L2_cache_block_invalidate(start, end);
#define keystone_rio_data_sync_rw(start, end)    L2_cache_block_writeback_invalidate(start, end);
#else /* CONFIG_TMS320C6X_CACHES_ON */
#define keystone_rio_data_sync_write(start, end)
#define keystone_rio_data_sync_read(start, end)
#define keystone_rio_data_sync_rw(start, end)
#endif /* CONFIG_TMS320C6X_CACHES_ON */
#endif /* __KERNEL__*/

/*
 * C6670/8 RapidIO SerDes registers
 */ 
#define KEYSTONE_RIO_SERDES_REG_BASE      0x02620000
#define KEYSTONE_RIO_SERDES_STS_REG       0x154
#define KEYSTONE_RIO_SERDES_CFG_PLL_REG   0x360

#define KEYSTONE_RIO_SERDES_CFG_RX_REG(port) (0x364 + ((port) << 3))
#define KEYSTONE_RIO_SERDES_CFG_TX_REG(port) (0x368 + ((port) << 3))

/* 
 * C6670/8 RapidIO Control Registers 
 */
#define KEYSTONE_RIO_REG_BASE             0x02900000
#define KEYSTONE_RIO_REG_SIZE             0x00021000

#define KEYSTONE_RIO_PCR                  0x04
#define KEYSTONE_RIO_PER_SET_CNTL         0x14
#define KEYSTONE_RIO_PER_SET_CNTL1        0x18

#define KEYSTONE_RIO_GBL_EN_REG           0x24
#define KEYSTONE_RIO_GBL_EN_STAT_REG      0x28
#define KEYSTONE_RIO_BLK0_EN_REG          0x2c
#define KEYSTONE_RIO_BLK0_EN_STAT_REG     0x30
#define KEYSTONE_RIO_BLK1_EN_REG          0x34
#define KEYSTONE_RIO_BLK1_EN_STAT_REG     0x38
#define KEYSTONE_RIO_BLK2_EN_REG          0x3c
#define KEYSTONE_RIO_BLK2_EN_STAT_REG     0x40
#define KEYSTONE_RIO_BLK3_EN_REG          0x44
#define KEYSTONE_RIO_BLK3_EN_STAT_REG     0x48
#define KEYSTONE_RIO_BLK4_EN_REG          0x4c
#define KEYSTONE_RIO_BLK4_EN_STAT_REG     0x50
#define KEYSTONE_RIO_BLK5_EN_REG          0x54
#define KEYSTONE_RIO_BLK5_EN_STAT_REG     0x58
#define KEYSTONE_RIO_BLK6_EN_REG          0x5c
#define KEYSTONE_RIO_BLK6_EN_STAT_REG     0x60
#define KEYSTONE_RIO_BLK7_EN_REG          0x64
#define KEYSTONE_RIO_BLK7_EN_STAT_REG     0x68
#define KEYSTONE_RIO_BLK8_EN_REG          0x6c
#define KEYSTONE_RIO_BLK8_EN_STAT_REG     0x70
#define KEYSTONE_RIO_BLK9_EN_REG          0x74
#define KEYSTONE_RIO_BLK9_EN_STAT_REG     0x78

#define KEYSTONE_RIO_BLK_EN_REG(idx)      (0x2c + ((idx) << 3))
#define KEYSTONE_RIO_BLK_EN_STAT_REG(idx) (0x30 + ((idx) << 3))

#define KEYSTONE_RIO_MULTI_ID_REG1        0x80
#define KEYSTONE_RIO_MULTI_ID_REG2        0x84
#define KEYSTONE_RIO_MULTI_ID_REG3        0x88
#define KEYSTONE_RIO_MULTI_ID_REG4        0x8c
#define KEYSTONE_RIO_MULTI_ID_REG5        0x80
#define KEYSTONE_RIO_MULTI_ID_REG6        0x84
#define KEYSTONE_RIO_MULTI_ID_REG7        0x88
#define KEYSTONE_RIO_MULTI_ID_REG8        0x8c

#define KEYSTONE_RIO_PF16B_CNTL(idx)      (0x0e0 + ((idx) << 3))
#define KEYSTONE_RIO_PF8B_CNTL(idx)       (0x0e4 + ((idx) << 3))

/* Interrupt management */
#define KEYSTONE_RIO_DOORBELL0_ICSR       0x180
#define KEYSTONE_RIO_DOORBELL0_ICCR       0x188
#define KEYSTONE_RIO_DOORBELL1_ICSR       0x190
#define KEYSTONE_RIO_DOORBELL1_ICCR       0x198
#define KEYSTONE_RIO_DOORBELL2_ICSR       0x1a0
#define KEYSTONE_RIO_DOORBELL2_ICCR       0x1a8
#define KEYSTONE_RIO_DOORBELL3_ICSR       0x1b0
#define KEYSTONE_RIO_DOORBELL3_ICCR       0x1b8
#define KEYSTONE_RIO_LSU0_ICSR            0x1c0
#define KEYSTONE_RIO_LSU0_ICCR            0x1c8
#define KEYSTONE_RIO_LSU1_ICSR            0x1d0
#define KEYSTONE_RIO_LSU1_ICCR            0x1d8
#define KEYSTONE_RIO_ERR_RST_EVNT_ICSR    0x1e0
#define KEYSTONE_RIO_ERR_RST_EVNT_ICCR    0x1e8
#define KEYSTONE_RIO_DOORBELL0_ICRR       0x200
#define KEYSTONE_RIO_DOORBELL0_ICRR2      0x204
#define KEYSTONE_RIO_DOORBELL1_ICRR       0x20c
#define KEYSTONE_RIO_DOORBELL1_ICRR2      0x210
#define KEYSTONE_RIO_DOORBELL2_ICRR       0x218
#define KEYSTONE_RIO_DOORBELL2_ICRR2      0x21c
#define KEYSTONE_RIO_DOORBELL3_ICRR       0x224
#define KEYSTONE_RIO_DOORBELL3_ICRR2      0x228
#define KEYSTONE_RIO_LSU0_ICRR0           0x230
#define KEYSTONE_RIO_LSU0_ICRR1           0x234
#define KEYSTONE_RIO_LSU0_ICRR2           0x238
#define KEYSTONE_RIO_LSU0_ICRR3           0x23c
#define KEYSTONE_RIO_LSU1_ICRR0           0x240
#define KEYSTONE_RIO_ERR_RST_EVNT_ICRR    0x250
#define KEYSTONE_RIO_ERR_RST_EVNT_ICRR2   0x254
#define KEYSTONE_RIO_ERR_RST_EVNT_ICRR3   0x258
#define KEYSTONE_RIO_INTERRUPT_CTL        0x264

#define KEYSTONE_RIO_INTDST_RATE_CNTL(idx) (0x2d0 + ((idx) << 2))

/* RXU for type 11 */
#define KEYSTONE_RIO_RXU_MAP_L_REG(idx)   (0x400 + (0xc * (idx)))
#define KEYSTONE_RIO_RXU_MAP_H_REG(idx)   (0x404 + (0xc * (idx)))
#define KEYSTONE_RIO_RXU_MAP_QID_REG(idx) (0x408 + (0xc * (idx)))

#define KEYSTONE_RIO_RXU_MAP_START        0x400
#define KEYSTONE_RIO_RXU_MAP_END          0x700

/* RXU for type 9 */
#define KEYSTONE_RIO_RXU_T9_MAP_0_REG(idx) (0x700 + (0xc * (idx)))
#define KEYSTONE_RIO_RXU_T9_MAP_1_REG(idx) (0x704 + (0xc * (idx)))
#define KEYSTONE_RIO_RXU_T9_MAP_2_REG(idx) (0x708 + (0xc * (idx)))

#define KEYSTONE_RIO_RXU_T9_MAP_START     0x700
#define KEYSTONE_RIO_RXU_T9_MAP_END       0xa00

/* LSU */
#define KEYSTONE_RIO_LSU_REG0(lsu)        (0xd00 + (0x1c * (lsu)))
#define KEYSTONE_RIO_LSU_REG1(lsu)        (0xd04 + (0x1c * (lsu)))
#define KEYSTONE_RIO_LSU_REG2(lsu)        (0xd08 + (0x1c * (lsu)))
#define KEYSTONE_RIO_LSU_REG3(lsu)        (0xd0c + (0x1c * (lsu)))
#define KEYSTONE_RIO_LSU_REG4(lsu)        (0xd10 + (0x1c * (lsu)))
#define KEYSTONE_RIO_LSU_REG5(lsu)        (0xd14 + (0x1c * (lsu)))
#define KEYSTONE_RIO_LSU_REG6(lsu)        (0xd18 + (0x1c * (lsu)))

#define KEYSTONE_RIO_LSU_SETUP_REG0       0xde0
#define KEYSTONE_RIO_LSU_SETUP_REG1       0xde4

#define KEYSTONE_RIO_LSU_STAT_REG(idx)    (0xde8 + ((idx) << 2))

/* Flow */
#define KEYSTONE_RIO_LSU_FLOW_MASKS0      0xe00
#define KEYSTONE_RIO_LSU_FLOW_MASKS1      0xe04
#define KEYSTONE_RIO_LSU_FLOW_MASKS2      0xe08
#define KEYSTONE_RIO_LSU_FLOW_MASKS3      0xe0c

#define KEYSTONE_RIO_FLOW_CNTL(idx)       (0xe50 + ((idx) << 2))

#define KEYSTONE_RIO_TX_CPPI_FLOW_MASK(idx) (0xeb0 + ((idx) << 2))

/* RapidIO Peripheral-Specific Registers (configuration space) */
#define KEYSTONE_RIO_CONF_SPACE           0xb000
#define KEYSTONE_RIO_DEV_ID               0xb000
#define KEYSTONE_RIO_DEV_INFO             0xb004
#define KEYSTONE_RIO_ASBLY_ID             0xb008
#define KEYSTONE_RIO_ASBLY_INFO           0xb00c
#define KEYSTONE_RIO_PE_FEAT              0xb010
#define KEYSTONE_RIO_SW_PORT              0xb014
#define KEYSTONE_RIO_SRC_OP               0xb018
#define KEYSTONE_RIO_DEST_OP              0xb01c
#define KEYSTONE_RIO_PE_LL_CTL            0xb04c
#define KEYSTONE_RIO_LCL_CFG_HBAR         0xb058
#define KEYSTONE_RIO_LCL_CFG_BAR          0xb05c
#define KEYSTONE_RIO_BASE_ID              0xb060

/* RapidIO Extended Features - LP Serial Registers */
#define KEYSTONE_RIO_SP_MB_HEAD           0xb100
#define KEYSTONE_RIO_SP_LT_CTL            0xb120
#define KEYSTONE_RIO_SP_RT_CTL            0xb124
#define KEYSTONE_RIO_SP_GEN_CTL           0xb13c

#define KEYSTONE_RIO_SP_LM_REQ(port)      (0xb140 + ((port) << 5))
#define KEYSTONE_RIO_SP_LM_RESP(port)     (0xb144 + ((port) << 5))
#define KEYSTONE_RIO_SP_ACKID_STAT(port)  (0xb148 + ((port) << 5))
#define KEYSTONE_RIO_SP_CTL2(port)        (0xb154 + ((port) << 5))
#define KEYSTONE_RIO_SP_ERR_STAT(port)    (0xb158 + ((port) << 5))
#define KEYSTONE_RIO_SP_CTL(port)         (0xb15c + ((port) << 5))

/* Error management registers */
#define KEYSTONE_RIO_ERR_RPT_BH           0xc000
#define KEYSTONE_RIO_ERR_DET              0xc008
#define KEYSTONE_RIO_ERR_EN               0xc00c
#define KEYSTONE_RIO_H_ADDR_CAPT          0xc010
#define KEYSTONE_RIO_ADDR_CAPT            0xc014
#define KEYSTONE_RIO_ID_CAPT              0xc018
#define KEYSTONE_RIO_CTRL_CAPT            0xc01c

#define KEYSTONE_RIO_SP_ERR_DET(port)     (0xc040 + ((port) << 6))
#define KEYSTONE_RIO_SP_RATE_EN(port)     (0xc044 + ((port) << 6))

/* Physical layer implementation specific registers (PLM) */
#define KEYSTONE_RIO_SP_IMP_SPEC_CTL(port)    (0x1b080 + ((port) << 7))
#define KEYSTONE_RIO_SP_PWDN_CTL(port)        (0x1b084 + ((port) << 7))
#define KEYSTONE_RIO_SP_PLM_STATUS(port)      (0x1b090 + ((port) << 7))
#define KEYSTONE_RIO_SP_INT_ENABLE(port)      (0x1b094 + ((port) << 7))
#define KEYSTONE_RIO_SP_ALL_INT_EN(port)      (0x1b0a0 + ((port) << 7))
#define KEYSTONE_RIO_SP_PATH_CTL(port)        (0x1b0b0 + ((port) << 7))
#define KEYSTONE_RIO_SP_DISCOVERY_TIMER(port) (0x1b0b4 + ((port) << 7))
#define KEYSTONE_RIO_SP_SILENCE_TIMER(port)   (0x1b0b8 + ((port) << 7))
#define KEYSTONE_RIO_SP_LONG_CS_TX1(port)     (0x1b0e0 + ((port) << 7))
#define KEYSTONE_RIO_SP_LONG_CS_TX2(port)     (0x1b0e4 + ((port) << 7))

/* Transport Layer specific registers (TLM) */
#define KEYSTONE_RIO_TLM_SP_CONTROL(port)     (0x1b380 + ((port) << 7))

/* Event manager specific registers */
#define KEYSTONE_RIO_EM_RST_PORT_STAT     0x1b960
#define KEYSTONE_RIO_EM_RST_INT_EN        0x1b968
#define KEYSTONE_RIO_EM_RST_PW_EN         0x1b970

/* Port-Write specific registers */
#define KEYSTONE_RIO_PW_CTL               0x1ba04
#define KEYSTONE_RIO_PW_RX_STAT           0x1ba10
#define KEYSTONE_RIO_PW_RX_CAPT(idx)      (0x1ba20 + ((idx) << 2))

/* Misc. registers */
#define KEYSTONE_RIO_PRESCALAR_SRV_CLK    0x1bd30
#define KEYSTONE_RIO_RST_CTL              0x1bd34

/*
 * Packet types
 */
#define KEYSTONE_RIO_PACKET_TYPE_NREAD    0x24
#define KEYSTONE_RIO_PACKET_TYPE_NWRITE   0x54
#define KEYSTONE_RIO_PACKET_TYPE_NWRITE_R 0x55
#define KEYSTONE_RIO_PACKET_TYPE_SWRITE   0x60
#define KEYSTONE_RIO_PACKET_TYPE_DBELL    0xa0
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_R  0x80
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_W  0x81
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_RR 0x82
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_WR 0x83
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_PW 0x84

/*
 * LSU defines
 */
#define KEYSTONE_RIO_LSU_PRIO             0

#define KEYSTONE_RIO_LSU_BUSY_MASK        0x80000000
#define KEYSTONE_RIO_LSU_FULL_MASK        0x40000000

#define KEYSTONE_RIO_LSU_CC_MASK          0x0f
#define KEYSTONE_RIO_LSU_CC_TIMEOUT       0x01
#define KEYSTONE_RIO_LSU_CC_XOFF          0x02
#define KEYSTONE_RIO_LSU_CC_ERROR         0x03
#define KEYSTONE_RIO_LSU_CC_INVALID       0x04
#define KEYSTONE_RIO_LSU_CC_DMA           0x05
#define KEYSTONE_RIO_LSU_CC_RETRY         0x06
#define KEYSTONE_RIO_LSU_CC_CANCELED      0x07

/* Mask for receiving both error and good completion LSU interrupts */
#define KEYSTONE_RIO_ICSR_LSU0(src_id)    ((0x10001) << (src_id))

/*
 * Various RIO defines
 */
#define KEYSTONE_RIO_DBELL_NUMBER         4
#define KEYSTONE_RIO_DBELL_VALUE_MAX      (KEYSTONE_RIO_DBELL_NUMBER * 16)
#define KEYSTONE_RIO_DBELL_MASK           (KEYSTONE_RIO_DBELL_VALUE_MAX - 1)

#define KEYSTONE_RIO_TIMEOUT_CNT          1000
#define KEYSTONE_RIO_TIMEOUT_MSEC         100
#define KEYSTONE_RIO_TIMEOUT_NSEC         1000
#define KEYSTONE_RIO_RETRY_CNT            4

/* 
 * Default configuration for SP_IP_MODE CSR
 * Apparently the F8_TGT_ID_DIS field is not correctly read, so it needs
 * to set it explicitely for each write to the SP_IP_MODE CSR
 */
#define KEYSTONE_RIO_SP_IP_MODE_DEFAULT   0x4d000000 /* SP_MODE = 1, PW_DIS, SCR_TGT_ID_DIS, F8_TGT_ID_DIS */

/*
 * RIO error, reset and special event interrupt defines
 */
#define KEYSTONE_RIO_ERR_RST_EVNT_MASK    0x00010f07
#define KEYSTONE_RIO_PORT_ERROR_MASK      0x07120214

#define KEYSTONE_RIO_RESET_INT            16  /* device reset interrupt on any port */
#define KEYSTONE_RIO_PORT3_ERROR_INT      11  /* port 3 error */
#define KEYSTONE_RIO_PORT2_ERROR_INT      10  /* port 2 error */
#define KEYSTONE_RIO_PORT1_ERROR_INT      9   /* port 1 error */
#define KEYSTONE_RIO_PORT0_ERROR_INT      8   /* port 0 error */
#define KEYSTONE_RIO_EVT_CAP_ERROR_INT    2   /* logical layer error management event capture */
#define KEYSTONE_RIO_PORT_WRITEIN_INT     1   /* port-write-in request received on any port */
#define KEYSTONE_RIO_MCAST_EVT_INT        0   /* multi-cast event control symbol interrupt received on any port */

/*
 * PktDMA definitions
 */
#define KEYSTONE_RIO_CDMA_BASE			0x02901000
#define KEYSTONE_RIO_CDMA_SIZE			0x1400
#define KEYSTONE_RIO_CDMA_GLOBAL_CFG_OFFSET	0x0000
#define KEYSTONE_RIO_CDMA_TX_CHAN_CFG_OFFSET	0x0400
#define KEYSTONE_RIO_CDMA_RX_CHAN_CFG_OFFSET	0x0800
#define KEYSTONE_RIO_CDMA_TX_SCH_CFG_OFFSET	0x0C00
#define KEYSTONE_RIO_CDMA_RX_FLOW_CFG_OFFSET	0x1000

#define KEYSTONE_RIO_CDMA_RX_FIRST_CHANNEL 	0
#define KEYSTONE_RIO_CDMA_RX_NUM_CHANNELS	16
#define KEYSTONE_RIO_CDMA_RX_FIRST_FLOW 	0
#define KEYSTONE_RIO_CDMA_RX_NUM_FLOWS		20
#define KEYSTONE_RIO_CDMA_RX_TIMEOUT_COUNT	1000
#define KEYSTONE_RIO_CDMA_TX_FIRST_CHANNEL 	0
#define KEYSTONE_RIO_CDMA_TX_NUM_CHANNELS	16

/*
 * RIO message passing defines
 */
#define KEYSTONE_RIO_MAX_MBOX             4    /* 4 in multi-segment, 64 in single-segment */

#define KEYSTONE_RIO_MSG_DESC_SIZE        16
#define KEYSTONE_RIO_MSG_MAX_BUFFER_SIZE  4096
#define KEYSTONE_RIO_MSG_SSIZE            0xe

#define KEYSTONE_RIO_MAP_FLAG_SEGMENT     (1 << 0)
#define KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC (1 << 1)
#define KEYSTONE_RIO_MAP_FLAG_TT_16       (1 << 13)
#define KEYSTONE_RIO_MAP_FLAG_DST_PROMISC (1 << 15)

#define KEYSTONE_RIO_DESC_FLAG_TT_16      (1 << 9)

#define KEYSTONE_RIO_LOOP_FREE_QUEUE      20 

/* 
 * RapidIO global definitions
 */
#define KEYSTONE_RIO_MAX_PORT             4
#define KEYSTONE_RIO_MAX_DIO_PKT_SIZE     0x100000 /* hardware support up to 1MB */

/*
 * Block definition
 */ 
#define KEYSTONE_RIO_BLK_NUM              9

#define KEYSTONE_RIO_BLK_MMR              0
#define KEYSTONE_RIO_BLK_LSU              1
#define KEYSTONE_RIO_BLK_MAU              2
#define KEYSTONE_RIO_BLK_TXU              3
#define KEYSTONE_RIO_BLK_RXU              4
#define KEYSTONE_RIO_BLK_PORT0            5
#define KEYSTONE_RIO_BLK_PORT1            6
#define KEYSTONE_RIO_BLK_PORT2            7
#define KEYSTONE_RIO_BLK_PORT3            8

#ifdef __KERNEL__
#include <asm/dscr.h>
#include <asm/irq.h>

/*
 * Interrupt and DMA event mapping
 * MP RXU and TXU interrupts are provided in platform_data
 */
#define KEYSTONE_LSU_RIO_INT              0            /* RIO interrupt used for LSU (global) */
#define KEYSTONE_GEN_RIO_INT              1            /* RIO interrupt used for generic RIO events (local) */

#define KEYSTONE_LSU_RIO_EVT              IRQ_RIOINT0  /* RIO interrupt used for LSU (global) */
#define KEYSTONE_GEN_RIO_EVT              IRQ_RIOINT1  /* RIO interrupt used for generic RIO events (local) */

/*
 * Dev Id and dev revision
 */
#define KEYSTONE_RIO_DEV_ID_VAL           ((((dscr_get_reg(DSCR_JTAGID)) << 4)  & 0xffff0000) | 0x30)
#define KEYSTONE_RIO_DEV_INFO_VAL         (((dscr_get_reg(DSCR_JTAGID)) >> 28) & 0xf)

#endif /* __KERNEL__*/
#endif /* _MACH_RIO_C667X_H */
