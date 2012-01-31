/*
 *  linux/arch/c6x/platforms/include/mach/rio.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef _MACH_RIO_TCI648X_H
#define _MACH_RIO_TCI648X_H

#include <asm/hardware.h>
#include <asm/io.h>
#include <mach/board.h>

/* 
 * TCI6487/8 RapidIO Control Registers 
 */
#define TCI648X_RIO_REG_BASE             0x02d00000
#define TCI648X_RIO_REG_SIZE             0x00021000

#define TCI648X_RIO_DESC_BASE            0x02e00000
#define TCI648X_RIO_DESC_SIZE            0x00004000

#define TCI648X_RIO_PCR                  0x04
#define TCI648X_RIO_PER_SET_CNTL         0x20

#define TCI648X_RIO_DEVICEID_REG1        0x80
#define TCI648X_RIO_DEVICEID_REG2        0x84
#define TCI648X_RIO_DEVICEID_REG3        0x88
#define TCI648X_RIO_DEVICEID_REG4        0x8c

#define TCI648X_RIO_PF16B_CNTL0          0x90
#define TCI648X_RIO_PF8B_CNTL0           0x94
#define TCI648X_RIO_PF16B_CNTL1          0x98
#define TCI648X_RIO_PF8B_CNTL1           0x9c

#define TCI648X_RIO_SERDES_CFGRX0_CNTL   0x100
#define TCI648X_RIO_SERDES_CFGRX1_CNTL   0x104
#define TCI648X_RIO_SERDES_CFGRX2_CNTL   0x108
#define TCI648X_RIO_SERDES_CFGRX3_CNTL   0x10c
#define TCI648X_RIO_SERDES_CFGTX0_CNTL   0x110
#define TCI648X_RIO_SERDES_CFGTX1_CNTL   0x114
#define TCI648X_RIO_SERDES_CFGTX2_CNTL   0x118
#define TCI648X_RIO_SERDES_CFGTX3_CNTL   0x11c
#define TCI648X_RIO_SERDES_CFG0_CNTL     0x120
#define TCI648X_RIO_SERDES_CFG1_CNTL     0x124
#define TCI648X_RIO_SERDES_CFG2_CNTL     0x128
#define TCI648X_RIO_SERDES_CFG3_CNTL     0x12c

#define TCI648X_RIO_DOORBELL0_ICSR       0x200
#define TCI648X_RIO_DOORBELL0_ICCR       0x208
#define TCI648X_RIO_DOORBELL1_ICSR       0x210
#define TCI648X_RIO_DOORBELL1_ICCR       0x218
#define TCI648X_RIO_DOORBELL2_ICSR       0x220
#define TCI648X_RIO_DOORBELL2_ICCR       0x228
#define TCI648X_RIO_DOORBELL3_ICSR       0x230
#define TCI648X_RIO_DOORBELL3_ICCR       0x238
#define TCI648X_RIO_RX_CPPI_ICSR         0x240
#define TCI648X_RIO_RX_CPPI_ICCR         0x248
#define TCI648X_RIO_TX_CPPI_ICSR         0x250
#define TCI648X_RIO_TX_CPPI_ICCR         0x258
#define TCI648X_RIO_LSU_ICSR             0x260
#define TCI648X_RIO_LSU_ICCR             0x268
#define TCI648X_RIO_ERR_RST_EVNT_ICSR    0x270
#define TCI648X_RIO_ERR_RST_EVNT_ICCR    0x278
#define TCI648X_RIO_DOORBELL0_ICRR       0x280
#define TCI648X_RIO_DOORBELL0_ICRR2      0x284
#define TCI648X_RIO_DOORBELL1_ICRR       0x290
#define TCI648X_RIO_DOORBELL1_ICRR2      0x294
#define TCI648X_RIO_DOORBELL2_ICRR       0x2a0
#define TCI648X_RIO_DOORBELL2_ICRR2      0x2a4
#define TCI648X_RIO_DOORBELL3_ICRR       0x2b0
#define TCI648X_RIO_DOORBELL3_ICRR2      0x2b4
#define TCI648X_RIO_RX_CPPI_ICRR         0x2c0
#define TCI648X_RIO_RX_CPPI_ICRR2        0x2c4
#define TCI648X_RIO_TX_CPPI_ICRR         0x2d0
#define TCI648X_RIO_TX_CPPI_ICRR2        0x2d4
#define TCI648X_RIO_LSU_ICRR0            0x2e0
#define TCI648X_RIO_LSU_ICRR1            0x2e4
#define TCI648X_RIO_LSU_ICRR2            0x2e8
#define TCI648X_RIO_LSU_ICRR3            0x2ec
#define TCI648X_RIO_ERR_RST_EVNT_ICRR    0x2f0
#define TCI648X_RIO_ERR_RST_EVNT_ICRR2   0x2f4
#define TCI648X_RIO_ERR_RST_EVNT_ICRR3   0x2f8

#define TCI648X_RIO_INTDST0_RATE_CNTL    0x320
#define TCI648X_RIO_INTDST1_RATE_CNTL    0x324
#define TCI648X_RIO_INTDST2_RATE_CNTL    0x328
#define TCI648X_RIO_INTDST3_RATE_CNTL    0x32c
#define TCI648X_RIO_INTDST4_RATE_CNTL    0x330
#define TCI648X_RIO_INTDST5_RATE_CNTL    0x334
#define TCI648X_RIO_INTDST6_RATE_CNTL    0x338
#define TCI648X_RIO_INTDST7_RATE_CNTL    0x33c

#define TCI648X_RIO_LSU1_REG0            0x400
#define TCI648X_RIO_LSU1_REG1            0x404
#define TCI648X_RIO_LSU1_REG2            0x408
#define TCI648X_RIO_LSU1_REG3            0x40c
#define TCI648X_RIO_LSU1_REG4            0x410
#define TCI648X_RIO_LSU1_REG5            0x414
#define TCI648X_RIO_LSU1_REG6            0x418
#define TCI648X_RIO_LSU1_FLOW_MASKS      0x41c

#define TCI648X_RIO_TXDMA_HDP0           0x500
#define TCI648X_RIO_TXDMA_CP0            0x580
#define TCI648X_RIO_RXDMA_HDP0           0x600
#define TCI648X_RIO_RXDMA_CP0            0x680
#define TCI648X_RIO_TX_QUEUE_TEARDOWN    0x700
#define TCI648X_RIO_TX_CPPI_FLOW_MASK0   0x704
#define TCI648X_RIO_RX_QUEUE_TEARDOWN    0x740
#define TCI648X_RIO_RX_CPPI_CNTL         0x744
#define TCI648X_RIO_TX_QUEUE_CNTL0       0x7e0

#define TCI648X_RIO_RXU_MAP_START        0x800
#define TCI648X_RIO_RXU_MAP_END          0x900

#define TCI648X_RIO_FLOW_CNTL            0x900

/* RapidIO Peripheral-Specific Registers (configuration space) */
#define TCI648X_RIO_CONF_SPACE           0x1000
#define TCI648X_RIO_DEV_ID               0x1000
#define TCI648X_RIO_DEV_INFO             0x1004
#define TCI648X_RIO_ASBLY_ID             0x1008
#define TCI648X_RIO_ASBLY_INFO           0x100c
#define TCI648X_RIO_PE_FEAT              0x1010
#define TCI648X_RIO_SRC_OP               0x1018
#define TCI648X_RIO_DEST_OP              0x101c
#define TCI648X_RIO_PE_LL_CTL            0x104c
#define TCI648X_RIO_LCL_CFG_HBAR         0x1058
#define TCI648X_RIO_LCL_CFG_BAR          0x105c
#define TCI648X_RIO_BASE_ID              0x1060

#define TCI648X_RIO_ERR_DET              0x2008
#define TCI648X_RIO_ERR_EN               0x200c
#define TCI648X_RIO_H_ADDR_CAPT          0x2010
#define TCI648X_RIO_ADDR_CAPT            0x2014
#define TCI648X_RIO_ID_CAPT              0x2018
#define TCI648X_RIO_CTRL_CAPT            0x201c

/* RapidIO Extended Features - LP Serial Registers */
#define TCI648X_RIO_SP_MB_HEAD           0x1100
#define TCI648X_RIO_SP_LT_CTL            0x1120
#define TCI648X_RIO_SP_RT_CTL            0x1124
#define TCI648X_RIO_SP_GEN_CTL           0x113c
#define TCI648X_RIO_SP0_ERR_STAT         0x1158
#define TCI648X_RIO_SP0_CTL              0x115c
#define TCI648X_RIO_SP1_ERR_STAT         0x1178
#define TCI648X_RIO_SP1_CTL              0x117c

#define TCI648X_RIO_SP0_CTL_INDEP        0x14004
#define TCI648X_RIO_SP1_CTL_INDEP        0x14104

#define TCI648X_RIO_SP_IP_DISCOV_TIMER   0x12000
#define TCI648X_RIO_SP_IP_MODE           0x12004
#define TCI648X_RIO_IP_PRESCAL           0x12008
#define TCI648X_RIO_SP_IP_PW_IN_CAPT0    0x12010
#define TCI648X_RIO_SP_IP_PW_IN_CAPT1    0x12014
#define TCI648X_RIO_SP_IP_PW_IN_CAPT2    0x12018
#define TCI648X_RIO_SP_IP_PW_IN_CAPT3    0x1201c

#define TCI648X_RIO_SP0_SILENCE_TIMER    0x14008

/*
 * Packet types
 */
#define TCI648X_RIO_PACKET_TYPE_NREAD    0x24
#define TCI648X_RIO_PACKET_TYPE_NWRITE   0x54
#define TCI648X_RIO_PACKET_TYPE_NWRITE_R 0x55
#define TCI648X_RIO_PACKET_TYPE_SWRITE   0x60
#define TCI648X_RIO_PACKET_TYPE_DBELL    0xa0
#define TCI648X_RIO_PACKET_TYPE_MAINT_R  0x80
#define TCI648X_RIO_PACKET_TYPE_MAINT_W  0x81
#define TCI648X_RIO_PACKET_TYPE_MAINT_RR 0x82
#define TCI648X_RIO_PACKET_TYPE_MAINT_WR 0x83
#define TCI648X_RIO_PACKET_TYPE_MAINT_PW 0x84

/*
 * LSU defines
 */
#define TCI648X_RIO_LSU_BUSY_MASK        0x01
#define TCI648X_RIO_LSU_CC_MASK          0x1e
#define TCI648X_RIO_LSU_CC_TIMEOUT       0x02
#define TCI648X_RIO_LSU_CC_XOFF          0x04
#define TCI648X_RIO_LSU_CC_ERROR         0x06
#define TCI648X_RIO_LSU_CC_INVALID       0x08
#define TCI648X_RIO_LSU_CC_DMA           0x0a
#define TCI648X_RIO_LSU_CC_RETRY         0x0c
#define TCI648X_RIO_LSU_CC_CREDIT        0x0e

#define TCI648X_RIO_ICSR_LSU1            0x000000ff
#define TCI648X_RIO_ICSR_LSU2            0x0000ff00
#define TCI648X_RIO_ICSR_LSU3            0x00ff0000
#define TCI648X_RIO_ICSR_LSU4            0xff000000

/*
 * Various RIO defines
 */
#define TCI648X_RIO_DBELL_NUMBER         4
#define TCI648X_RIO_DBELL_VALUE_MAX      (TCI648X_RIO_DBELL_NUMBER * 16)
#define TCI648X_RIO_DBELL_MASK           (TCI648X_RIO_DBELL_VALUE_MAX - 1)

#define TCI648X_RIO_TIMEOUT              1000
#define TCI648X_RIO_RETRY_SHIFT_FACTOR   2

/* 
 * Default configuration for SP_IP_MODE CSR
 * Apparently the F8_TGT_ID_DIS field is not correctly read, so it needs
 * to set it explicitely for each write to the SP_IP_MODE CSR
 */
#define TCI648X_RIO_SP_IP_MODE_DEFAULT   0x4d000000 /* SP_MODE = 1, PW_DIS, SCR_TGT_ID_DIS, F8_TGT_ID_DIS */

/*
 * RIO error, reset and special event interrupt defines
 */
#define TCI648X_RIO_ERR_RST_EVNT_MASK    0x00010f07
#define TCI648X_RIO_PORT_ERROR_MASK      0x03110004

#define TCI648X_RIO_RESET_INT            16  /* device reset interrupt on any port */
#define TCI648X_RIO_PORT3_ERROR_INT      11  /* port 3 error */
#define TCI648X_RIO_PORT2_ERROR_INT      10  /* port 2 error */
#define TCI648X_RIO_PORT1_ERROR_INT      9   /* port 1 error */
#define TCI648X_RIO_PORT0_ERROR_INT      8   /* port 0 error */
#define TCI648X_RIO_EVT_CAP_ERROR_INT    2   /* logical layer error management event capture */
#define TCI648X_RIO_PORT_WRITEIN_INT     1   /* port-write-in request received on any port */
#define TCI648X_RIO_MCAST_EVT_INT        0   /* multi-cast event control symbol interrupt received on any port */

/*
 * RIO message passing defines
 */
#define TCI648X_MAX_MBOX                 4    /* 4 in multi-segment, 64 in single-segment */
#define TCI648X_RIO_MSG_RX_QUEUE_FIRST   0    /* first hw queue used for Rx, from 0 to 15 */
#define TCI648X_RIO_MSG_TX_QUEUE_FIRST   0    /* first hw queue used for Tx, from 0 to 15 */
#define TCI648X_RIO_MSG_RX_QUEUE_END     3    /* last hw queue used for Rx, from 0 to 15 */
#define TCI648X_RIO_MSG_TX_QUEUE_END     3    /* last hw queue used for Tx, from 0 to 15 */

#define TCI648X_RIO_MSG_DESC_SIZE	 16
#define TCI648X_RIO_MSG_MAX_BUFFER_SIZE	 4096
#define TCI648X_RIO_MSG_BUFFER_SIZE	 1552 /* to contain one Ethernet packet (1514) + reserve + alignement */
#define TCI648X_RIO_MSG_SSIZE            0xe
#define TCI648X_RIO_MIN_RING_SIZE        2
#define TCI648X_RIO_MAX_RING_SIZE        2048
#define TCI648X_RIO_MIN_TX_RING_SIZE	 TCI648X_RIO_MIN_RING_SIZE
#define TCI648X_RIO_MAX_TX_RING_SIZE	 TCI648X_RIO_MAX_RING_SIZE
#define TCI648X_RIO_MIN_RX_RING_SIZE	 TCI648X_RIO_MIN_RING_SIZE
#define TCI648X_RIO_MAX_RX_RING_SIZE	 TCI648X_RIO_MAX_RING_SIZE

#define TCI648X_RIO_DESC_FLAG_CC         (7 << 9)
#define TCI648X_RIO_DESC_FLAG_TT_16      (1 << 12)
#define TCI648X_RIO_DESC_FLAG_TEARDOWN   (1 << 27)
#define TCI648X_RIO_DESC_FLAG_EOQ        (1 << 28)
#define TCI648X_RIO_DESC_FLAG_OWNER      (1 << 29)
#define TCI648X_RIO_DESC_FLAG_EOP        (1 << 30)
#define TCI648X_RIO_DESC_FLAG_SOP        (1 << 31)

#define TCI648X_RIO_MAP_FLAG_SEGMENT     (1 << 0)
#define TCI648X_RIO_MAP_FLAG_PROMISC     (1 << 1)
#define TCI648X_RIO_MAP_FLAG_TT_16       (1 << 8)

/* 
 * RapidIO modes (SERDES configuration)
 */
#define TCI648X_RIO_MODE_0               0x00000 /* BOOTMODE 8 config */
#define TCI648X_RIO_MODE_1               0x00001 /* BOOTMODE 9 config */
#define TCI648X_RIO_MODE_2               0x00002 /* BOOTMODE 10 config */
#define TCI648X_RIO_MODE_3               0x00003 /* BOOTMODE 11 config */

#define TCI648X_RIO_MAX_PORT             2
#define TCI648X_RIO_MAX_DIO_PKT_SIZE     0x1000

/*
 * Interrupt and DMA event mapping
 */
#ifdef CONFIG_SOC_TMS320C6474

#define TCI648X_LSU_RIO_INT              0  /* RIO interrupt used for LSU */
#define TCI648X_RXTX_RIO_INT             1  /* RIO interrupt used for Rx/Tx */
#define TCI648X_EDMA_RIO_INT             2  /* RIO interrupt used for EDMA */

#define TCI648X_LSU_RIO_EVT              IRQ_RIOINT0  /* RIO interrupt used for LSU */
#define TCI648X_RXTX_RIO_EVT             IRQ_RIOINT1  /* RIO interrupt used for Rx/Tx */

#define TCI648X_LSU_CHANNEL_EVENT        DMA_CIC_EVT6 /* Dummy */
#define TCI648X_ICCR_CHANNEL_EVENT       DMA_CIC_EVT5 /* mapped to RIO interrupt 2 via the CIC */
#define TCI648X_RATE_CHANNEL_EVENT       DMA_CIC_EVT7 /* Dummy */

/* Map the RIO event 2 to the proper CIC TPCC (CIC3) event (DMA_CIC_EVT5)*/
#define tci648x_map_rio_edma_event()     cic_raw_map(CIC_TPCC_RIOINT2, CIC5, CIC_TPCC);

#endif /* CONFIG_SOC_TMS320C6474 */

#ifdef CONFIG_SOC_TMS320C6472

#define TCI648X_LSU_RIO_INT              6  /* RIO interrupt used for LSU (global) */
#define TCI648X_RXTX_RIO_INT             (0 + get_coreid())  /* RIO interrupt used for Rx/Tx (local) */
#define TCI648X_EDMA_RIO_INT             7  /* RIO interrupt used for EDMA (global) */

#define TCI648X_LSU_RIO_EVT              IRQ_RIOINT6  /* RIO interrupt used for LSU (global) */
#define TCI648X_RXTX_RIO_EVT             IRQ_RIOINT   /* RIO interrupt used for Rx/Tx (local) */

#define TCI648X_LSU_CHANNEL_EVENT        DMA_UNUSED   /* Dummy */
#define TCI648X_ICCR_CHANNEL_EVENT       DMA_RIOINT7  /* Use RIO interrupt 7 (global) */
#define TCI648X_RATE_CHANNEL_EVENT       (DMA_UNUSED + 1) /* Dummy */

#define tci648x_map_rio_edma_event() 

#endif /* CONFIG_SOC_TMS320C6472 */

/*
 * This is per board configuration
 */
#define TCI648X_LSU_MACH_INT             IRQ_RIO_LSU    /* DSP interrupt for LSU */
#define TCI648X_RXTX_MACH_INT            IRQ_RIO_RXTX   /* DSP interrupt for Rx/Tx */

#ifdef __KERNEL__

#define TCI648X_RIO_GET_DID(x, m)	 (m ? (x & 0xffff) : ((x & 0x00ff0000) >> 16))
#define TCI648X_RIO_SET_DID(x, m)	 (m ? (x & 0xffff) : ((x & 0x000000ff) << 16))

#define DEVICE_REG32_W(x, y)              *(volatile unsigned int *)(x) = (y)
#define DEVICE_REG32_R(x)                (*(volatile unsigned int *)(x))

extern struct rio_dbell *rio_retrieve_dbell(u32 dbell_num);
#endif /* __KERNEL__*/

#endif /* _MACH_RIO_TCI648X_H */
