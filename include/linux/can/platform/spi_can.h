/*
 *  CAN over SPI Driver
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (a-jacquiot@ti.com)
 *
 */
#ifndef __ASM_SPI_CAN_H__
#define __ASM_SPI_CAN_H__

#define QUEUE_RUNNING             0
#define QUEUE_STOPPED             1

#define CAN_TO_DEV(can)           (&(can)->spi->dev)

/*
 * Commands for the SPI protocol
 */
#define SPI_CAN_CMD_HEADER	  0xcd00
#define SPI_CAN_CMD_MEM_WRITE	  0xcd00
#define SPI_CAN_CMD_MEM_READ	  0xcd01
#define SPI_CAN_CMD_CAN_TX        0xcd10
#define SPI_CAN_CMD_CAN_RX        0xcd11
#define SPI_CAN_CMD_CAN_BTSET     0xcd12
#define SPI_CAN_CMD_CAN_TX_INT    0xcd20
#define SPI_CAN_CMD_CAN_ERROR     0xcd21
#define SPI_CAN_CMD_NOP           0xcdf0
#define SPI_CAN_CMD_READY	  0xcdfe
#define SPI_CAN_CMD_DUMMY	  0xcdff


/*
 * CAN controller errors
 */
#define SPI_CAN_STATUS_LEC_STUFF  0x0001
#define SPI_CAN_STATUS_LEC_FORM   0x0002
#define SPI_CAN_STATUS_LEC_ACK    0x0003
#define SPI_CAN_STATUS_LEC_BIT1   0x0004
#define SPI_CAN_STATUS_LEC_BIT0   0x0005
#define SPI_CAN_STATUS_LEC_CRC    0x0006
#define SPI_CAN_STATUS_EPASS      0x0020
#define SPI_CAN_STATUS_EWARN      0x0040
#define SPI_CAN_STATUS_BUS_OFF    0x0080

/*
 * Device driver structures
 */
struct spi_can_queue {
	struct workqueue_struct  *workqueue;
	struct work_struct        work;
	struct sk_buff           *skb;
	int                       len;
	int                       run;
};

/*
 * Format of SPI frames
 */

struct spi_can_c_frame {
	canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	__u8    can_dlc; /* data length code: 0 .. 8 */
	__u8    data[8];
};

struct spi_can_mem_frame {
	u32                       addr;
	u32                       data;
};

struct spi_can_bt_frame {
	u32                       bitrate;              
	u16                       sync_prop_phase1_seg; 
	u16                       phase2_seg;
	u16                       sjw;
	u16                       quantum_prescaler;
};

struct spi_can_frame {
	u16                       cmd;
	u16                       size;
	union {
		struct spi_can_c_frame    cf;
		struct spi_can_bt_frame   bf;
		struct spi_can_mem_frame  mf;
	} u;
};

#define SPI_CAN_FRAME_HEADER_LEN  4

struct spi_can_priv {
	struct can_priv           can;	   /* must be the first member */
	struct net_device_stats   stats;   /* must be the second member */
	struct net_device        *ndev;

	struct spi_device        *spi;
        struct spi_can_frame     *tx_buf;  /* SPI Rx/Tx buffers */ 
        struct spi_can_frame     *rx_buf;
        u16                      *tx_ack;
        u16                      *rx_ack;
        struct spi_can_frame     *tx_dummy;
        struct spi_can_frame     *rx_dummy;

	struct spi_can_queue      tx_q;
	struct spi_can_queue      rx_q;

	struct semaphore          lock;

};

struct spi_can_message {
	struct list_head          queue;
	struct can_frame          cf;
};

#endif /*__ASM_SPI_CAN_H__ */
