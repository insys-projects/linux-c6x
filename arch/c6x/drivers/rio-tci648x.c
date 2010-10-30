/*
 *  linux/arch/c6x/drivers/rio-tci648x.c
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/smp_lock.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/ioport.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/kfifo.h>

#include <asm/setup.h>
#include <asm/virtconvert.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#ifdef CONFIG_EDMA3
#include <asm/edma.h>
#endif
#include <asm/cache.h>
#include <asm/io.h>
#include <asm/delay.h>

#include <asm/rio.h>

#undef TCI648X_RIO_DEBUG

#ifdef TCI648X_RIO_DEBUG
#define DPRINTK(fmt, args...) printk(KERN_DEBUG "RIO: [%s] " fmt, __FUNCTION__ , ## args)
#define ASSERT(cond) if (!(cond)) DPRINTK("ASSERT %s FAILURE\n", # cond)
#else
#define ASSERT(cond)
#define DPRINTK(fmt, args...) 
#endif

static char banner[] __initdata = KERN_INFO "TCI648x RapidIO driver v2.1\n";

static u32 rio_regs;
static u32 rio_conf_regs;

struct port_write_msg {
	union rio_pw_msg msg;
	u32              msg_count;
	u32              err_count;
	u32              discard_count;
};

/*
 * Main TCI648x RapidIO driver data
 */
struct tci648x_rio_data {
	struct completion      lsu_completion;
	struct semaphore       lsu_lock;
#ifdef CONFIG_EDMA3
	int	               lsu_edma_ch;   /* LSU load EDMA channel */
	int	               iccr_edma_ch;  /* ICCR load EDMA channel */
	int	               rate_edma_ch;  /* RATE load EDMA channel */
	edmacc_paramentry_regs lsu_edma_params;
	edmacc_paramentry_regs iccr_edma_params;
	edmacc_paramentry_regs rate_edma_params;
	int                    dummy_edma_ch; /* dummy EDMA for int. gen. */
	int                    iccr_tcc; 
	int                    rate_tcc;
#endif
	wait_queue_head_t      dbell_waitq[TCI648X_RIO_DBELL_VALUE_MAX];
	spinlock_t             dbell_i_lock;
	struct port_write_msg  port_write_msg;
	struct work_struct     pw_work;
	struct kfifo           pw_fifo;
	spinlock_t             pw_fifo_lock;
};

static struct tci648x_rio_data _tci648x_rio;

/*
 * SERDES configuration for a RapidIO controller
 */
struct tci648x_serdes_config {
	u16 macro_config;                         /* SERDES macro configuration */
	u32 rx_chan_config[TCI648X_RIO_MAX_PORT]; /* SERDES receive channel configuration (per-port) */
	u32 tx_chan_config[TCI648X_RIO_MAX_PORT]; /* SERDES transmit channel configuration (per-port) */
};

static struct tci648x_serdes_config _tci648x_serdes_config[4] = {

        /* sRIO config 0, BOOTMODE 8:  
	   SERDES ref clock: 125 MHz, Link rate = 1.25 Gbps */
	{ 0x000B,
	  { 0x00081121, 0x00081121 },
	  { 0x00000921, 0x00000921 } },

	/* sRIO config 1, BOOTMODE 9:
	   SERDES ref clock: 125 MHz, Link rate = 3.125 Gbps */
	{ 0x000F,
	  { 0x00081101, 0x00081101 },
	  { 0x00000901, 0x00000901 } },

	/* sRIO config 2, BOOTMODE 10:
	   SERDES ef clock: 156.25 MHz, Link rate = 1.25 Gbps */
	{ 0x0009,
	  { 0x00081121, 0x00081121 },
	  { 0x00000921, 0x00000921 } },

	/* sRIO config 3, BOOTMODE 11:
	   SERDES ref clock: 156.25 MHz, Link rate = 3.125 Gbps */
	{ 0x000B,
	  { 0x00081101, 0x00081101 },
	  { 0x00000901, 0x00000901 } }
};

static void dbell_handler(struct tci648x_rio_data *p_rio);
static void tci648x_rio_port_write_handler(struct tci648x_rio_data *p_rio);
static void cppi_tx_handler(u32 queue);
static void cppi_rx_handler(u32 queue);

#define index_to_port(index) (index)
#define port_to_index(port)  (port)
#define is_power_of_2(x)     (((x) & ((x) - 1)) == 0)

struct rio_msg_desc {
	struct rio_msg_desc *next;
	u8                  *pbuff;
	u32                  opt1;
	u32                  opt2;
};

struct tci648x_rio_msg_ring {
	u32                  slot;
	int                  count;
	u32                  entries;
	u32                  error;
	u32                  running;
	u32                  full;
       	struct rio_msg_desc *desc_base;
	struct rio_msg_desc *dirty;
	struct rio_msg_desc *head;
	void                *dev_id;
	u32                  payload;
	u32                  queue;
	u32                  mbox;
	struct rio_mport    *port;
	struct resource      desc_res;
	struct resource      map_res;
	spinlock_t           lock;
};

#define TCI648X_RIO_MSG_RX_QUEUE_NUM (TCI648X_RIO_MSG_RX_QUEUE_END + 1)
#define TCI648X_RIO_MSG_TX_QUEUE_NUM (TCI648X_RIO_MSG_TX_QUEUE_END + 1)

static struct tci648x_rio_msg_ring  msg_rx_ring[TCI648X_RIO_MSG_RX_QUEUE_NUM];
static struct tci648x_rio_msg_ring  msg_tx_ring[TCI648X_RIO_MSG_TX_QUEUE_NUM];

static struct resource              _tci648x_rio_desc_res;
static struct resource              _tci648x_rio_rxu_map_res;

#define get_cur_desc(desc)          ((desc)->desc_base + (desc)->slot)
#define get_prev_desc(desc, p_ring) ((desc) == (p_ring)->desc_base ? \
				     ((p_ring)->desc_base + (p_ring)->entries - 1) : (desc) - 1)

/* Priority for LSU transfer */
#define LSU_PRIO                    2 

#ifdef CONFIG_EDMA3
/*
 * Threshold size to start using EDMA instead of CPU for DIO
 * By default, EDMA is never used. 
 * You can change this with the rioedma-threshold= command line parameter.
 */
static int tci648x_edma_threshold = -1;
#endif

/*------------------------- RapidIO hw controller setup ---------------------*/

/**
 * tci648x_rio_hw_init - Configure a RapidIO controller
 * @mode: serdes configuration
 * @hostid: device id of the host
 */
static void tci648x_rio_hw_init(u32 mode, u16 hostid, u32 size)
{
	u32 dummy;

	/* SERDES configuration */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_SERDES_CFG0_CNTL,
		       _tci648x_serdes_config[mode].macro_config);

	__delay(30000);

	/* Alllow writing read-only registers */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_PER_SET_CNTL, 0x0C053910);

	/* Peripheral-specific configuration and capabilities */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_DEV_ID,     0x00920030);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_DEV_INFO,   0x00000004);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_ASBLY_ID,   0x00000030);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_ASBLY_INFO, 0x00000100);

	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_PE_FEAT, 
		       RIO_PEF_PROCESSOR
		       | RIO_PEF_CTLS
		       | RIO_PEF_FLOW_CONTROL
		       | RIO_PEF_EXT_FEATURES 
		       | RIO_PEF_ADDR_34
		       | RIO_PEF_INB_DOORBELL
		       | RIO_PEF_INB_MBOX);

	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_SRC_OP,       
		       RIO_SRC_OPS_READ
		       | RIO_SRC_OPS_WRITE
		       | RIO_SRC_OPS_STREAM_WRITE
		       | RIO_SRC_OPS_WRITE_RESPONSE
		       | RIO_SRC_OPS_DATA_MSG
		       | RIO_SRC_OPS_DOORBELL
		       | RIO_SRC_OPS_ATOMIC_TST_SWP
		       | RIO_SRC_OPS_ATOMIC_INC
		       | RIO_SRC_OPS_ATOMIC_DEC
		       | RIO_SRC_OPS_ATOMIC_SET
		       | RIO_SRC_OPS_ATOMIC_CLR
		       | RIO_SRC_OPS_PORT_WRITE);

	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_DEST_OP,
		       RIO_DST_OPS_READ
		       | RIO_DST_OPS_WRITE
		       | RIO_DST_OPS_STREAM_WRITE
		       | RIO_DST_OPS_WRITE_RESPONSE
		       | RIO_DST_OPS_DATA_MSG
		       | RIO_DST_OPS_DOORBELL
		       | RIO_DST_OPS_PORT_WRITE);

	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_PE_LL_CTL,
		       RIO_PELL_ADDR_34);

	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_MB_HEAD,   0x10000002);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LCL_CFG_HBAR, 0x00000000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LCL_CFG_BAR,  0x005A0200);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_GEN_CTL,   0x00000000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_LT_CTL,    0xFFFFFF00);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_RT_CTL,    0xFFFFFF00);

        /* Set base ID */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_BASE_ID,
		       TCI648X_RIO_SET_DID(hostid, size));
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_DEVICEID_REG1,
		       TCI648X_RIO_SET_DID(hostid, size));

	/* Port IP mode, SRC_TGT_ID_DIS is set */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_DISCOV_TIMER,
		       0x90800000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_MODE,   0x4D00003F);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_IP_PRESCAL,   0x00000008);

	/* Set error detection mode */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_ERR_DET,      0x00000000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_ERR_EN,       0x00000000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_H_ADDR_CAPT,  0x00000000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_ADDR_CAPT,    0x00000000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_ID_CAPT,      0x00000000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_CTRL_CAPT,    0x00000000);

	/* Force all writes to finish */
	dummy = DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_IP_PRESCAL);
}

/**
 * tci648x_rio_start - Start RapidIO controller
 */
static void tci648x_rio_start(void)
{
	/* Control register: LOG_TGT_ID_DIS, BOOT_COMPLETE is set */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_PER_SET_CNTL, 0x0D053910);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_PCR,          0x00000005);

	/* Needed, otherwise it will not be able to send requests out */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_GEN_CTL,   0x60000000);
}

/**
 * tci648x_add_flow_control - Add flow control for a given device ID
 */
static int tci648x_add_flow_control(struct rio_dev *rdev)
{
	static int free_entry = 0;
	u32        flow_mask;
	int        queue;

	if (free_entry == 16)
		return -ENOMEM;

	/* Setup flow control */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_FLOW_CNTL + (free_entry << 2),
		       rdev->net->hport->sys_size ? 0x10000 | rdev->destid : rdev->destid);

	DPRINTK("adding flow control entry %d for devid %d\n", free_entry, rdev->destid);

	/* Enable control flow for LSU1 */
	flow_mask  = DEVICE_REG32_R(TCI648X_RIO_REG_BASE
				    + TCI648X_RIO_LSU1_FLOW_MASKS);
	flow_mask |= 1 << free_entry;
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_FLOW_MASKS, 
		       flow_mask);

	DPRINTK("LSU flow mask 0x%x\n", flow_mask);
	
	/* Enable control flow for CPPI Tx */
	queue = rdev->net->hport->id + TCI648X_RIO_MSG_TX_QUEUE_FIRST;

	flow_mask  = DEVICE_REG32_R(TCI648X_RIO_REG_BASE
				    + TCI648X_RIO_TX_CPPI_FLOW_MASK0
				    + (queue & ~0x1));
	flow_mask |= 1 << (free_entry + ((queue & 0x1) << 4));
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE
		       + TCI648X_RIO_TX_CPPI_FLOW_MASK0
		       + (queue & ~0x1), flow_mask);

	DPRINTK("TX queue %d flow mask 0x%x\n", queue, flow_mask);

	free_entry++;

	return 0;
}

/**
 * tci648x_rio_port_status - Return if the port is OK or not
 * @port: index of the port
 *
 * Return %0 if the port is ready or %-EIO on failure.
 */
static int tci648x_rio_port_status(int port)
{
	unsigned int count, value, portok;

	count = 0;
	portok= 0;

	if (port >= TCI648X_RIO_MAX_PORT)
		return -EINVAL;

	while(1) { 
		value =	DEVICE_REG32_R(TCI648X_RIO_REG_BASE
				       + TCI648X_RIO_SP0_ERR_STAT + (port << 5));
		
		if ((value & RIO_PORT_N_ERR_STS_PORT_OK) !=0) {
			portok++;
			if (portok >= 50) 
				break; /* port must be solid OK */
			__delay(100);
		} else {
			portok = 0;
			count++;
			if (count >= TCI648X_RIO_TIMEOUT) 
				return -EIO;
		}
		__delay(100);
	}
	return 0;
}

/**
 * tci648x_rio_port_init - Configure a RapidIO port
 * @port: index of the port to configure
 * @mode: serdes configuration
 */
static int tci648x_rio_port_init(u32 port, u32 mode)
{
	u32 dummy;
	u32 idx = (port << 2);

	if (port >= TCI648X_RIO_MAX_PORT)
		return -EINVAL;

	/* Per-port SERDES configuration */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + (TCI648X_RIO_SERDES_CFGRX0_CNTL + idx),
		       _tci648x_serdes_config[mode].rx_chan_config[port]);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + (TCI648X_RIO_SERDES_CFGTX0_CNTL + idx),
		       _tci648x_serdes_config[mode].tx_chan_config[port]);

	/* Disable packet forwarding */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + (TCI648X_RIO_PF16B_CNTL0 + (idx << 1)),
		       0xffffffff); 
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + (TCI648X_RIO_PF8B_CNTL0 + (idx << 1)),
		       0x0003ffff);

	/* Silence timer */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + (TCI648X_RIO_SP0_SILENCE_TIMER + (idx << 6)),
		       0xB0000000);

	/* Port control */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + (TCI648X_RIO_SP0_CTL + (idx << 3)),
		       0x00600001);  /* Port is serial and enabled */

	return 0;
}

static int tci648x_rio_init(void) 
{
	unsigned int i;
	int          res;

	for (i = 0; i < TCI648X_RIO_DBELL_VALUE_MAX; i++)
		init_waitqueue_head(&_tci648x_rio.dbell_waitq[i]);

	mutex_init(&_tci648x_rio.lsu_lock);
	init_completion(&_tci648x_rio.lsu_completion);

        /* RIO configuration space */
	rio_regs      = (u32) ioremap(TCI648X_RIO_REG_BASE, TCI648X_RIO_REG_SIZE);
	rio_conf_regs = rio_regs + TCI648X_RIO_CONF_SPACE; 

	/* Initialize MP descriptor resources  */ 
	memset(&_tci648x_rio_desc_res, 0, sizeof(struct resource));
        _tci648x_rio_desc_res.start = TCI648X_RIO_DESC_BASE;
        _tci648x_rio_desc_res.end   = TCI648X_RIO_DESC_BASE + TCI648X_RIO_DESC_SIZE;
	_tci648x_rio_desc_res.flags = RIO_RESOURCE_MEM;

	/* Initialize MP RXU map table resources */
	memset(&_tci648x_rio_rxu_map_res, 0, sizeof(struct resource));
	_tci648x_rio_rxu_map_res.start = TCI648X_RIO_REG_BASE + TCI648X_RIO_RXU_MAP_START;
	_tci648x_rio_rxu_map_res.end   = TCI648X_RIO_REG_BASE + TCI648X_RIO_RXU_MAP_END;
	_tci648x_rio_rxu_map_res.flags = RIO_RESOURCE_MEM;

	/* Initialize interrupts */
	tci648x_rio_interrupt_setup();

	/* Init port write interface */
	res = tci648x_rio_port_write_init(&_tci648x_rio);
	if (res)
		goto out;

#ifdef CONFIG_EDMA3	
	/* EDMA setup */
	res = tci648x_rio_edma_setup();
#endif

out:
	return res;
}

static int tci648x_rio_release(void) {

	tci648x_rio_interrupt_release();
#ifdef CONFIG_EDMA3	
	tci648x_rio_edma_release();
#endif
	return 0;
}

/*----------------------------------- EDMA management ------------------------------*/
#ifdef CONFIG_EDMA3
static void lsu_edma_callback(int lch, u16 ch_status, void *data)
{
	struct tci648x_rio_data *p_rio = (struct tci648x_rio_data *) data;

	complete(&p_rio->lsu_completion);
}

static int tci648x_rio_edma_setup(void)
{
	int                    tcc = -1;

	/* Request the LSU load channel */ 
	if (request_edma(TCI648X_LSU_CHANNEL_EVENT,
			 "sRIO LSU",
			 lsu_edma_callback,
			 &_tci648x_rio,
			 &_tci648x_rio.lsu_edma_ch,
			 &tcc,
			 EVENTQ_2)) {
	        printk("Unable to request EDMA channel for sRIO LSU\n");
		return -EAGAIN;
	}

	/* Request the ICCR load channel */ 
	if (request_edma(TCI648X_ICCR_CHANNEL_EVENT,
			 "sRIO ICCR",
			 NULL,
			 NULL, 
			 &_tci648x_rio.iccr_edma_ch,
			 &tcc,
			 EVENTQ_2)) {
	        printk("Unable to request EDMA channel for sRIO ICCR\n");
		return -EAGAIN;
	}
	
	/* Request the RATE load channel */ 
	if (request_edma(TCI648X_RATE_CHANNEL_EVENT,
			 "sRIO RATE",
			 NULL,
			 NULL,
			 &_tci648x_rio.rate_edma_ch, 
			 &tcc,
			 EVENTQ_2)) {
	        printk("Unable to request EDMA channel for sRIO RATE\n");
		return -EAGAIN;
	}

	/* Eventually map the sRIO event to the EDMA event */
	tci648x_map_rio_edma_event();

	/* Just in case... */
	stop_edma(_tci648x_rio.lsu_edma_ch); 
	stop_edma(_tci648x_rio.iccr_edma_ch); 
	stop_edma(_tci648x_rio.rate_edma_ch); 

	/* Setup constant EDMA parameters */	
	set_edma_dest_index(_tci648x_rio.lsu_edma_ch, 0, 0);
	set_edma_src_index(_tci648x_rio.lsu_edma_ch, 
			   sizeof(struct tci648x_rio_lsu_reg), 0);

	set_edma_dest_params(_tci648x_rio.iccr_edma_ch,
			     TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU_ICCR,
			     0, 0);
	set_edma_dest_index(_tci648x_rio.iccr_edma_ch, 0, 0);
	set_edma_src_index(_tci648x_rio.iccr_edma_ch, 0, 0);

	set_edma_dest_params(_tci648x_rio.rate_edma_ch,
			     TCI648X_RIO_REG_BASE + TCI648X_RIO_INTDST0_RATE_CNTL
			     + (TCI648X_EDMA_RIO_INT << 2),
			     0, 0);
	set_edma_dest_index(_tci648x_rio.rate_edma_ch, 0, 0);
	set_edma_src_index(_tci648x_rio.rate_edma_ch, 0, 0);

	/* Setup chaining */
	edma_chain_lch(_tci648x_rio.iccr_edma_ch, _tci648x_rio.rate_edma_ch);
	edma_chain_lch(_tci648x_rio.rate_edma_ch, _tci648x_rio.lsu_edma_ch);

	/* Add intermediate event generation and set TCCMODE */
	get_edma_params(_tci648x_rio.lsu_edma_ch, &_tci648x_rio.lsu_edma_params);
	_tci648x_rio.lsu_edma_params.opt |= TCCMODE;
	set_edma_params(_tci648x_rio.lsu_edma_ch, &_tci648x_rio.lsu_edma_params);

	get_edma_params(_tci648x_rio.iccr_edma_ch, &_tci648x_rio.iccr_edma_params);
	_tci648x_rio.iccr_edma_params.opt |= ITCCHEN | TCCMODE;
	set_edma_params(_tci648x_rio.iccr_edma_ch, &_tci648x_rio.iccr_edma_params);

	get_edma_params(_tci648x_rio.rate_edma_ch, &_tci648x_rio.rate_edma_params);
	_tci648x_rio.rate_edma_params.opt |= ITCCHEN;
	set_edma_params(_tci648x_rio.rate_edma_ch, &_tci648x_rio.rate_edma_params);

	return 0;
}

static int tci648x_rio_edma_release(void)
{
	stop_edma(_tci648x_rio.lsu_edma_ch); 
	stop_edma(_tci648x_rio.iccr_edma_ch); 
	stop_edma(_tci648x_rio.rate_edma_ch); 

	free_edma(_tci648x_rio.lsu_edma_ch);
	free_edma(_tci648x_rio.iccr_edma_ch);
	free_edma(_tci648x_rio.rate_edma_ch);

	return 0;
}
#endif /* CONFIG_EDMA3 */
/*--------------------------------- Interrupt management ----------------------------*/

static irqreturn_t lsu_interrupt_handler(int irq, void *data)
{
	struct tci648x_rio_data *p_rio = (struct tci648x_data *) data;
	u32 pending_lsu_int            =
		DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU_ICSR);

	if (pending_lsu_int & TCI648X_RIO_ICSR_LSU1) {
		complete(&p_rio->lsu_completion);
	}

	/* ACK the interrupt */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU_ICCR, pending_lsu_int);

	/* Re-arm interrupt */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_INTDST0_RATE_CNTL
		       + (TCI648X_LSU_RIO_INT << 2), 0);

	return IRQ_HANDLED;
}

static void special_interrupt_handler(int ics)
{
	u32 reg;
	u32 port;
	u32 error;
	
	/* Acknowledge the interrupt */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_ERR_RST_EVNT_ICCR, 1 << ics);

	switch(ics) {
	case TCI648X_RIO_MCAST_EVT_INT:
		/* Multi-cast event control symbol interrupt received on any port */
		reg = DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_MODE);
		DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_MODE,
			       reg | TCI648X_RIO_SP_IP_MODE_DEFAULT | (1 << 4));
		break;

	case TCI648X_RIO_PORT_WRITEIN_INT:
		/* Port-write-in request received on any port */
		tci648x_rio_port_write_handler(&_tci648x_rio);
		break;

	case TCI648X_RIO_EVT_CAP_ERROR_INT:
		/* Logical layer error management event capture */
		break;

	case TCI648X_RIO_PORT0_ERROR_INT:
	case TCI648X_RIO_PORT1_ERROR_INT:
	case TCI648X_RIO_PORT2_ERROR_INT:
	case TCI648X_RIO_PORT3_ERROR_INT:
		/* Port error */
		port   = (ics - TCI648X_RIO_PORT0_ERROR_INT);

		error  = DEVICE_REG32_R(TCI648X_RIO_REG_BASE
					+ TCI648X_RIO_SP0_ERR_STAT + (port << 5))
			& ~TCI648X_RIO_PORT_ERROR_MASK;

		DEVICE_REG32_W(TCI648X_RIO_REG_BASE
			       + TCI648X_RIO_SP0_ERR_STAT + (port << 5), error);

		reg = DEVICE_REG32_R(TCI648X_RIO_REG_BASE
				     + TCI648X_RIO_SP0_CTL_INDEP + (port << 8));
		DEVICE_REG32_W(TCI648X_RIO_REG_BASE
			       + TCI648X_RIO_SP0_CTL_INDEP + (port << 8),
			       reg | (1 << 6));
		break;
			
	case TCI648X_RIO_RESET_INT:
		/* Device reset interrupt on any port */
		reg = DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_MODE);
		DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_MODE,
			       reg | TCI648X_RIO_SP_IP_MODE_DEFAULT | (1 << 2));
		break;

	}
	return;
}

static irqreturn_t rxtx_interrupt_handler(int irq, void *data)
{
	struct tci648x_rio_data *p_rio = (struct tci648x_rio_data *) data;
	u32 pending_rx_cppi_int        = 
		DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_RX_CPPI_ICSR);
	u32 pending_err_rst_evnt_int   =
		DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_ERR_RST_EVNT_ICSR)
		& TCI648X_RIO_ERR_RST_EVNT_MASK;
	u32 pending_tx_cppi_int        =
		DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_TX_CPPI_ICSR);

	/* Handle special interrupts (error, reset, special event) */
	while (pending_err_rst_evnt_int) {
		u32 ics = __ffs(pending_err_rst_evnt_int); 
		pending_err_rst_evnt_int &= ~(1 << ics);
		special_interrupt_handler(ics);
	}

	/* Call doorbell handler */
	dbell_handler(p_rio);

	/* Check CPPI Rx interrupt */
	while (pending_rx_cppi_int) {
		u32 queue = __ffs(pending_rx_cppi_int);
		pending_rx_cppi_int &= ~(1 << queue);

		if ((queue >= TCI648X_RIO_MSG_RX_QUEUE_FIRST) &&
		    (queue <= TCI648X_RIO_MSG_RX_QUEUE_END)) {
			
			/* Call the CPPI Tx handler */
			cppi_rx_handler(queue);
		}
	}

	/* Check CPPI Tx interrupt */
	while (pending_tx_cppi_int) {
		u32 queue = __ffs(pending_tx_cppi_int);
		
		pending_tx_cppi_int &= ~(1 << queue);

		if ((queue >= TCI648X_RIO_MSG_TX_QUEUE_FIRST) &&
		    (queue <= TCI648X_RIO_MSG_TX_QUEUE_END)) {
			
			/* Call the CPPI Tx handler */
			cppi_tx_handler(queue);
		}
	}

	/* Re-arm the interrupt */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_INTDST0_RATE_CNTL
		       + (TCI648X_RXTX_RIO_INT << 2), 0);

	return IRQ_HANDLED;
}

/*
 * Map a sRIO event to a sRIO interrupt
 */
static void tci648x_rio_interrupt_map(u32 reg, u32 mask, u32 rio_int)
{
	int i;
	u32 reg_val;

	reg_val = DEVICE_REG32_R(reg);

	for (i = 0; i <= 32; i+= 4) {
		if ((mask >> i) & 0xf) {
			reg_val &= ~(0xf << i);
			reg_val |= (rio_int << i);
		}
	}
	DEVICE_REG32_W(reg, reg_val);
}

/*
 * Setup RIO interrupts
 */
static void tci648x_rio_interrupt_setup(void)
{	
	/* Clear all pending interrupts */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_DOORBELL0_ICCR,     0x0000ffff);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_DOORBELL1_ICCR,     0x0000ffff);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_DOORBELL2_ICCR,     0x0000ffff);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_DOORBELL3_ICCR,     0x0000ffff);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_RX_CPPI_ICCR,       0x0000ffff);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_TX_CPPI_ICCR,       0x0000ffff);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU_ICCR,           0xffffffff);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_ERR_RST_EVNT_ICCR,  0x00010f07);

	/* LSU interrupts are routed to RIO interrupt dest 0 (LSU) */
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU_ICRR0,          0x11111111, TCI648X_LSU_RIO_INT);
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU_ICRR1,          0x11111111, TCI648X_LSU_RIO_INT);
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU_ICRR2,          0x11111111, TCI648X_LSU_RIO_INT);
        tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU_ICRR3,          0x11111111, TCI648X_LSU_RIO_INT);
	
	/* CPPI Tx interrupts are routed to RIO interrupt dest 1 (Rx/Tx) */
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_TX_CPPI_ICRR,       0x11111111, TCI648X_RXTX_RIO_INT);
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_TX_CPPI_ICRR2,      0x11111111, TCI648X_RXTX_RIO_INT);

	/* Doorbell interrupts are routed to RIO interrupt dest 1 (Rx/Tx) */
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_DOORBELL0_ICRR,     0x11111111, TCI648X_RXTX_RIO_INT);
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_DOORBELL0_ICRR2,    0x11111111, TCI648X_RXTX_RIO_INT);
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_DOORBELL1_ICRR,     0x11111111, TCI648X_RXTX_RIO_INT);
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_DOORBELL1_ICRR2,    0x11111111, TCI648X_RXTX_RIO_INT);
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_DOORBELL2_ICRR,     0x11111111, TCI648X_RXTX_RIO_INT);
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_DOORBELL2_ICRR2,    0x11111111, TCI648X_RXTX_RIO_INT);
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_DOORBELL3_ICRR,     0x11111111, TCI648X_RXTX_RIO_INT);
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_DOORBELL3_ICRR2,    0x11111111, TCI648X_RXTX_RIO_INT);

	/* CPPI Rx interrupts are routed to RIO interrupt dest 1 (Rx/Tx) */
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_RX_CPPI_ICRR,       0x11111111, TCI648X_RXTX_RIO_INT);
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_RX_CPPI_ICRR2,      0x11111111, TCI648X_RXTX_RIO_INT);

	/* Error, reset and special event interrupts are routed to RIO interrupt dest 1 (Rx/Tx) */
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_ERR_RST_EVNT_ICRR,  0x00000111, TCI648X_RXTX_RIO_INT);
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_ERR_RST_EVNT_ICRR2, 0x00001111, TCI648X_RXTX_RIO_INT);
	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_ERR_RST_EVNT_ICRR3, 0x00000001, TCI648X_RXTX_RIO_INT);

	/* Do not use pacing */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_INTDST0_RATE_CNTL,  0x00000000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_INTDST1_RATE_CNTL,  0x00000000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_INTDST2_RATE_CNTL,  0x00000000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_INTDST3_RATE_CNTL,  0x00000000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_INTDST4_RATE_CNTL,  0x00000000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_INTDST5_RATE_CNTL,  0x00000000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_INTDST6_RATE_CNTL,  0x00000000);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_INTDST7_RATE_CNTL,  0x00000000);

	/* Map sRIO interrupt events to DSP interrupts */
	irq_map(TCI648X_LSU_RIO_EVT,  TCI648X_LSU_MACH_INT);
	irq_map(TCI648X_RXTX_RIO_EVT, TCI648X_RXTX_MACH_INT);

	/* Attach interrupt handlers */
	request_irq(TCI648X_RXTX_MACH_INT,
		    rxtx_interrupt_handler,
		    0,
		    "sRIO",
		    &_tci648x_rio);

	request_irq(TCI648X_LSU_MACH_INT,
		    lsu_interrupt_handler,
		    0,
		    "sRIO LSU",
		    &_tci648x_rio);

}

static void tci648x_rio_interrupt_release(void)
{
	free_irq(TCI648X_RXTX_MACH_INT, &_tci648x_rio);
	free_irq(TCI648X_LSU_MACH_INT, &_tci648x_rio);
}

/*---------------------------------- Direct I/O -------------------------------*/

static u32 tci648x_rio_dio_packet_type(int dio_mode)
{
	switch (dio_mode) {
	case RIO_DIO_MODE_READ:
		return TCI648X_RIO_PACKET_TYPE_NREAD;
	case RIO_DIO_MODE_WRITER:
		return TCI648X_RIO_PACKET_TYPE_NWRITE_R;
	case RIO_DIO_MODE_WRITE:
		return TCI648X_RIO_PACKET_TYPE_NWRITE;
	case RIO_DIO_MODE_SWRITE:
		return TCI648X_RIO_PACKET_TYPE_SWRITE;
	}
	return TCI648X_RIO_PACKET_TYPE_NREAD;
}

/*
 * DIO transfer using LSU directly 
 */
static inline int tci648x_rio_dio_raw_transfer(struct rio_mport *mport,
					       int index,
					       u16 dest_id,
					       u32 src_addr,
					       u32 tgt_addr,
					       int size_bytes,
					       int dio_mode)
{
	unsigned int count;
	unsigned int status      = 0;
	unsigned int res         = 0;
	unsigned int retry_count = 1 << TCI648X_RIO_RETRY_SHIFT_FACTOR;
retry_transfer:

	mutex_lock(&_tci648x_rio.lsu_lock);

	INIT_COMPLETION(_tci648x_rio.lsu_completion);

	/* LSU 1 Reg 0 - MSB of destination */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG0, 0);
	
	/* LSU 1 Reg 1 - LSB of destination */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG1, tgt_addr);
	
	/* LSU 1 Reg 2 - source address */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG2, src_addr);
	
	/* LSU 1 Reg 3 - Byte count */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG3, size_bytes);
	
	/* LSU 1 Reg 4 - 
	 * out port ID = rio.port
         * priority = LSU_PRIO
	 * XAM = 0
	 * ID size = 8 or 16 bit
	 * Dest ID specified as arg
	 * interrupt request = 1 */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG4,
		       ((index_to_port(index) << 30)
			| (LSU_PRIO << 28)
			| ((mport->sys_size) ? (1 << 24) : 0)
			| ((u32) dest_id << 8)
			| 1));
	
	/* LSU 1 Reg 5 -
	 * doorbell info = 0 for this packet type
	 * hop count = 0 for this packet type
	 * Writing this register should initiate the transfer */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG5, 
		       tci648x_rio_dio_packet_type(dio_mode));
	
        /* Wait for transfer to complete */
   	count = 0;
	res   = 0;
	while(1) {
                /* wait for interrupt */
		wait_for_completion(&_tci648x_rio.lsu_completion);

		/* check if LSU has really finished and status */
		status = DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG6);
		if ((status & TCI648X_RIO_LSU_BUSY_MASK) == 0)
			break;
		count++;
		if (count >= TCI648X_RIO_TIMEOUT) {
			res = -EIO;
			break;
		}
	}
	mutex_unlock(&_tci648x_rio.lsu_lock);

	if (res)
		return res;

	DPRINTK("status = 0x%x\n", status);

	switch (status & TCI648X_RIO_LSU_CC_MASK) {
	case TCI648X_RIO_LSU_CC_TIMEOUT:
	case TCI648X_RIO_LSU_CC_XOFF:
	case TCI648X_RIO_LSU_CC_ERROR:
	case TCI648X_RIO_LSU_CC_INVALID:
	case TCI648X_RIO_LSU_CC_DMA:
		res = -EIO;
	case TCI648X_RIO_LSU_CC_RETRY:
	case TCI648X_RIO_LSU_CC_CREDIT:
		res = -EAGAIN;
	default:
	}

	/* 
	 * Try to transfer again in case of retry doorbell receive
	 *  or unavailable outbound credit.
	 */
	if ((res == -EAGAIN) && (retry_count-- > 0)) {
		__delay(1000);
		goto retry_transfer;
	}

	return res;
}

#ifdef CONFIG_EDMA3
/*
 * DIO transfer using EDMA
 */
static inline int tci648x_rio_dio_edma_transfer(struct rio_mport *mport,
						int index,
						u16 dest_id,
						u32 src_addr,
						u32 tgt_addr,
						int size_bytes,
						int dio_mode)
{
        struct tci648x_rio_lsu_reg *lsu_reg_queue;
        struct tci648x_rio_lsu_reg *current_lsu_reg_queue;

	unsigned int  last_count  = size_bytes & (TCI648X_RIO_MAX_DIO_PKT_SIZE - 1);
	unsigned int  count       = last_count ? (size_bytes >> 12) + 1 : (size_bytes >> 12);
	unsigned int  real_count  = count;
	unsigned int  status      = 0;
	unsigned int  retry_count = count << TCI648X_RIO_RETRY_SHIFT_FACTOR;
	int           res         = 0;
	unsigned int  lsu;
	unsigned int  offset;
	unsigned int *p_rate_val;
	unsigned int *p_iccr_val;

	/* Allocate memory for DMA transfer: all LSU + one PACE value */
	lsu_reg_queue = (struct tci648x_rio_lsu_reg *) kzalloc(
		L1_CACHE_ALIGN(sizeof(struct tci648x_rio_lsu_reg) * (count + 1)),
		GFP_KERNEL);
	
	if (lsu_reg_queue == NULL)
		return -ENOMEM;
	
	/* Prepare the LSU registers queue */
	offset = 0;
	for (lsu = 0; lsu < count; lsu++) {

		/* LSU 1 Reg 0 - MSB of destination */
		lsu_reg_queue[lsu].reg[0] = 0;
		
		/* LSU 1 Reg 1 - LSB of destination */
		lsu_reg_queue[lsu].reg[1] = tgt_addr + offset;

		/* LSU 1 Reg 2 - source address */
		lsu_reg_queue[lsu].reg[2] = src_addr + offset;
	
		/* LSU 1 Reg 3 - Byte count */
		if (last_count && (lsu == (count - 1)))
			lsu_reg_queue[lsu].reg[3] = last_count;
		else
			lsu_reg_queue[lsu].reg[3] = 0;

		/* LSU 1 Reg 4 - 
		 * out port ID = rio.port
		 * priority = LSU_PRIO
		 * XAM = 0
		 * ID size = 8 or 16 bit
		 * Dest ID specified as arg
		 * interrupt request = 1 */
		lsu_reg_queue[lsu].reg[4] =		
			((index_to_port(index) << 30)
			 | (LSU_PRIO << 28)
			 | ((mport->sys_size) ? (1 << 24) : 0)
			 | ((u32) dest_id << 8)
			 | 1);

		/* LSU 1 Reg 5 -
		 * doorbell info = 0 for this packet type
		 * hop count = 0 for this packet type
		 * Writing this register should initiate the transfer */
		lsu_reg_queue[lsu].reg[5] =
			tci648x_rio_dio_packet_type(dio_mode);

		offset += TCI648X_RIO_MAX_DIO_PKT_SIZE;
	}

	p_rate_val =  &lsu_reg_queue[count].reg[0];
	p_iccr_val =  &lsu_reg_queue[count].reg[1];

	*p_rate_val = 0;  /* RATE value */
	*p_iccr_val = 1;  /* ICCR value (harcoded for LSU1) */

	/* No coherency is assumed between EDMA and L2 cache */
	L2_cache_block_writeback((u32) lsu_reg_queue,
				 (u32) &lsu_reg_queue[count + 2].reg[0]);

	current_lsu_reg_queue = lsu_reg_queue;

retry_transfer:
	/* Setup transfers */
	set_edma_params(_tci648x_rio.lsu_edma_ch, &_tci648x_rio.lsu_edma_params);
	set_edma_src_params(_tci648x_rio.lsu_edma_ch, current_lsu_reg_queue, 0, 0);
	set_edma_dest_params(_tci648x_rio.lsu_edma_ch,
			     TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG0,
			     0, 0);
	set_edma_transfer_params(_tci648x_rio.lsu_edma_ch, 
				 sizeof(struct tci648x_rio_lsu_reg), /* ACNT */
				 count,     /* BCNT */
				 1,         /* CCNT */    
				 1,         /* BCNTRLD */
				 ASYNC);
	
	set_edma_params(_tci648x_rio.iccr_edma_ch, &_tci648x_rio.iccr_edma_params);
	set_edma_src_params(_tci648x_rio.iccr_edma_ch, p_iccr_val, 0, 0);
	set_edma_transfer_params(_tci648x_rio.iccr_edma_ch, 
				 4,         /* ACNT */
				 count,     /* BCNT */
				 1,         /* CCNT */    
				 1,         /* BCNTRLD */
				 ASYNC);

	set_edma_params(_tci648x_rio.rate_edma_ch, &_tci648x_rio.rate_edma_params);
	set_edma_src_params(_tci648x_rio.rate_edma_ch, p_rate_val, 0, 0);
	set_edma_transfer_params(_tci648x_rio.rate_edma_ch, 
				 4,         /* ACNT */
				 count,     /* BCNT */
				 1,         /* CCNT */    
				 1,         /* BCNTRLD */
				 ASYNC);

	mutex_lock(&_tci648x_rio.lsu_lock);

	INIT_COMPLETION(_tci648x_rio.lsu_completion);

	/* Clear pending LSU1 transmit interrupt if any */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU_ICCR, 0x00000001);

	/* Set LSU1 transmit interrupt to EDMA event */
      	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU_ICRR0,
				  0x00000001,
				  TCI648X_EDMA_RIO_INT);

	/* Start EDMA */
	start_edma(_tci648x_rio.rate_edma_ch);
	start_edma(_tci648x_rio.iccr_edma_ch);
	start_edma(_tci648x_rio.lsu_edma_ch);

	/* Launch transfer */
	edma_trigger_evt(_tci648x_rio.lsu_edma_ch);

	/* Wait for EDMA completion */
	wait_for_completion(&_tci648x_rio.lsu_completion);

        /* Wait for transfer to complete */
   	count = 0;
	res   = 0;
	while(1) {
		/* Check if LSU has really finished and status */
		status = DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG6);
		if ((status & TCI648X_RIO_LSU_BUSY_MASK) == 0)
			break;
		count++;
		if (count >= TCI648X_RIO_TIMEOUT) {
			res = -EIO;
			break;
		}
		__delay(100);
	}

	/* Set LSU1 transmit interrupt to EDMA event */
      	tci648x_rio_interrupt_map(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU_ICRR0,
				  0x00000001,
				  TCI648X_LSU_RIO_INT);

	/* Stop EDMA */
	stop_edma(_tci648x_rio.lsu_edma_ch);
	stop_edma(_tci648x_rio.iccr_edma_ch);
	stop_edma(_tci648x_rio.rate_edma_ch);

	mutex_unlock(&_tci648x_rio.lsu_lock);

	if (res)
		goto end_transfer;

	DPRINTK("status = 0x%x\n", status);

	switch (status & TCI648X_RIO_LSU_CC_MASK) {
	case TCI648X_RIO_LSU_CC_TIMEOUT:
	case TCI648X_RIO_LSU_CC_XOFF:
	case TCI648X_RIO_LSU_CC_ERROR:
	case TCI648X_RIO_LSU_CC_INVALID:
	case TCI648X_RIO_LSU_CC_DMA:
		res = -EIO;
	case TCI648X_RIO_LSU_CC_RETRY:
	case TCI648X_RIO_LSU_CC_CREDIT:
		res = -EAGAIN;
	default:
	}

	/* 
	 * Try to transfer again in case of retry doorbell receive
	 *  or unavailable outbound credit.
	 */
	if ((res == -EAGAIN) && (retry_count-- > 0)) {
		edmacc_paramentry_regs edma_params;

		__delay(1000);
		/* Restart from previous LSU reg data */
		get_edma_params(_tci648x_rio.lsu_edma_ch, &edma_params);
		if (edma_params.opt == NULL) {
			/* transfer is finished */
			current_lsu_reg_queue = lsu_reg_queue + (real_count - 1);
			count                 = 1;
		} else {
			/* transfer is still valid */
			current_lsu_reg_queue = ((struct tci648x_rio_lsu_reg *) edma_params.src) - 1;
			count                 = ((edma_params.a_b_cnt >> 16) & 0xffff) + 1;
		}
		goto retry_transfer;
	}

end_transfer:
	kfree(lsu_reg_queue);
	return res;
}
#endif /* CONFIG_EDMA3 */

/**
 * tci648x_rio_dio_transfer - Transfer bytes data from/to DSP address
 * to device ID's global address.
 * @mport: RapidIO master port info
 * @index: ID of the RapidIO interface
 * @dest_id: destination device id 
 * @src_addr: source (host) address
 * @tgt_addr: target global address
 * @size_bytes: size in bytes
 * @dio_mode: DIO transfer mode (write, write_r, swrite , read)
 *
 * Return %0 on success. Return %-EIO, %-EBUSY or %-EAGAIN on failure.
 */
static int tci648x_rio_dio_transfer(struct rio_mport *mport,
				    int index,
				    u16 dest_id,
				    u32 src_addr,
				    u32 tgt_addr,
				    int size_bytes,
				    int dio_mode)
{
	int count = size_bytes;
	int length;
	int res;

#ifdef CONFIG_TMS320C6X_CACHES_ON
	/* Flush caches if needed */
	if (dio_mode != RIO_DIO_MODE_READ)
		L2_cache_block_writeback((u32) src_addr,
					 (u32) src_addr + size_bytes);
#endif

#ifdef CONFIG_EDMA3
	/* Check if the transfer uses EDMA */ 
	if ((tci648x_edma_threshold >= 0) && (size_bytes >= tci648x_edma_threshold)) {
		res = tci648x_rio_dio_edma_transfer(mport,
						    index, 
						    dest_id,
						    src_addr,
						    tgt_addr,
						    size_bytes,
						    dio_mode);
		count = 0;
#else  /* CONFIG_EDMA3 */
	if (0) {
#endif /* CONFIG_EDMA3 */

	} else {
		u32 s_addr = src_addr;
		u32 t_addr = tgt_addr;

		/* Otherwise transfer packet by packet */
		while(count) {
			length = (count <= TCI648X_RIO_MAX_DIO_PKT_SIZE) ? 
				count : TCI648X_RIO_MAX_DIO_PKT_SIZE;
			
			res = tci648x_rio_dio_raw_transfer(mport,
							   index,
							   dest_id,
							   s_addr,
							   t_addr,
							   length,
							   dio_mode);
			if (res)
				break;
			
			s_addr += length;
			t_addr += length;

			count -= length;
		}
	}
	
#ifdef CONFIG_TMS320C6X_CACHES_ON
	/* Invalidate caches if needed */
	if (dio_mode == RIO_DIO_MODE_READ)
		L2_cache_block_invalidate((u32) src_addr,
					  (u32) src_addr + size_bytes - count);
#endif
	return res;
}

/*------------------------------ Doorbell management --------------------------*/

static inline int dbell_get(u16* pending)
{
	if (*pending) {
		int n = __ffs(*pending);
		*pending &= ~(1 << n);
		return n;
	} else 
		return -1;
	
}

static void dbell_handler(struct tci648x_rio_data *p_rio)
{
	u16                      pending_dbell;
	u16                      ack_dbell;
	unsigned int             i;
 
	for (i = 0; i < TCI648X_RIO_DBELL_NUMBER; i++) {
		pending_dbell = 
			(u16) DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_DOORBELL0_ICSR +
					     ((TCI648X_RIO_DOORBELL1_ICSR - TCI648X_RIO_DOORBELL0_ICSR) * i));
		ack_dbell = pending_dbell;
		
		while (pending_dbell) {
			unsigned int dbell_num  = dbell_get(&pending_dbell)+ (i << 4);
			struct rio_dbell *dbell = rio_retrieve_dbell(dbell_num);

			if (dbell && dbell->dinb)
				dbell->dinb(dbell->mport,
					    dbell->dev_id,
					    -1, /* we don't know the source Id */
					    dbell->mport->host_deviceid,
					    dbell_num);

			if (waitqueue_active(&p_rio->dbell_waitq[dbell_num]))
				wake_up_all(&p_rio->dbell_waitq[dbell_num]);
		}

		if (ack_dbell)
			/* ACK the known interrupt for this doorbell */
			DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_DOORBELL0_ICCR + 
				       ((TCI648X_RIO_DOORBELL1_ICSR - TCI648X_RIO_DOORBELL0_ICSR) * i),
				       (u32) ack_dbell);
	}
}

/**
 * tci648x_rio_dbell_wait - Wait a TCI648x doorbell message
 * @mport: RapidIO master port info
 * @data: 16-bit info field of RapidIO doorbell message
 *
 * Wait a TCI648x doorbell message.
 */
static int tci648x_rio_dbell_wait(struct rio_mport *mport, u16 data)
{
	unsigned long flags;
	u32           dbnum = data & TCI648X_RIO_DBELL_MASK;

	DECLARE_WAITQUEUE(wait, current);

	spin_lock_irqsave(&_tci648x_rio.dbell_i_lock, flags);

wait_dbell:
	add_wait_queue(&_tci648x_rio.dbell_waitq[dbnum], &wait);

	current->state = TASK_INTERRUPTIBLE;
	spin_unlock_irqrestore(&_tci648x_rio.dbell_i_lock, flags);

	schedule();
	
	spin_lock_irqsave(&_tci648x_rio.dbell_i_lock, flags);
	remove_wait_queue(&_tci648x_rio.dbell_waitq[dbnum], &wait);
	current->state = TASK_RUNNING;

	if (signal_pending(current)) {
		spin_unlock_irqrestore(&_tci648x_rio.dbell_i_lock, flags);
		return -ERESTARTSYS;
	}	
	
	DPRINTK("Receiving doorbell (info = %d)\n", dbnum);
	
	return dbnum;
}

/**
 * tci648x_rio_doorbell_send - Send a TCI648x doorbell message
 * @mport: RapidIO master port info
 * @index: ID of the RapidIO interface
 * @destid: device ID of target device
 * @num: doorbell number
 *
 * Sends a TCI648x doorbell message. Returns %0 on success or
 * %-EINVAL, %-EIO, %-EBUSY or %-EAGAIN on failure.
 */
static int tci648x_rio_dbell_send(struct rio_mport *mport,
				  int index, 
				  u16 dest_id, 
				  u16 num)
{
	unsigned int      count;
	unsigned int      status = 0;
	unsigned int      res    = 0;
	/* Transform doorbell number into info field */
	u16               info   = (num & 0xf) | (((num >> 4) & 0x3) << 5);

	DPRINTK("Sending doorbell (info = %d) to %x\n", info & TCI648X_RIO_DBELL_MASK, dest_id);

	mutex_lock(&_tci648x_rio.lsu_lock);

	INIT_COMPLETION(_tci648x_rio.lsu_completion);

	/* LSU 1 Reg 0 - MSB of destination */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG0, 0);

	/* LSU 1 Reg 1 - LSB of destination */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG1, 0);

	/* LSU 1 Reg 2 - source address */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG2, 0);

	/* LSU 1 Reg 3 - byte count */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG3, 0);

	/* LSU 1 Reg 4 - */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG4,
		       ((index_to_port(index) << 30)
			| (LSU_PRIO << 28)
			| ((mport->sys_size) ? (1 << 24) : 0)
			| ((u32) dest_id << 8)
			| 1));

	/* LSU 1 Reg 5 
	 * doorbell info = info
	 * hop count = 0
	 * Packet type = 0xa0 ftype = 10, ttype = 0 */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG5,
		       ((info & 0xffff) << 16) | (TCI648X_RIO_PACKET_TYPE_DBELL & 0xff));
	
       /* Wait for it to complete */
   	count = 0;
	while(1) {
                /* wait for interrupt */
		wait_for_completion(&_tci648x_rio.lsu_completion);

		/* check if LSU has really finished and status */
		status = DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG6);
		if ((status & TCI648X_RIO_LSU_BUSY_MASK) == 0)
			break;
		count++;
		if (count >= TCI648X_RIO_TIMEOUT) {
			res = -EIO;
			break;
		}
	}

	mutex_unlock(&_tci648x_rio.lsu_lock);

	if (res)
	    return res;

	DPRINTK("status = 0x%x\n", status);

	switch (status & TCI648X_RIO_LSU_CC_MASK) {
	case TCI648X_RIO_LSU_CC_TIMEOUT:
	case TCI648X_RIO_LSU_CC_XOFF:
	case TCI648X_RIO_LSU_CC_ERROR:
	case TCI648X_RIO_LSU_CC_INVALID:
	case TCI648X_RIO_LSU_CC_DMA:
		return -EIO;
	case TCI648X_RIO_LSU_CC_RETRY:
		return -EBUSY;
	case TCI648X_RIO_LSU_CC_CREDIT:
		return -EAGAIN;
	default:
	}

	DPRINTK("Doorbell sent\n");

	return 0;
}

/*---------------------- Maintenance Request Management  ---------------------*/

/**
 * maint_request - Perform a maintenance request
 * @mport: Master port implementing the inbound message unit
 * @index: ID of the RapidIO interface
 * @destid: destination ID of target device
 * @hopcount: hopcount for this request
 * @offset: offset in the RapidIO configuration space
 * @len: length of the data 
 * @paddr: physical address of the data on the host
 * @type: packet type
 *
 * Returns %0 on success or %-EINVAL, %-EIO, %-EAGAIN or %-EBUSY on failure.
 */
static inline int maint_request(struct rio_mport *mport,
				int index,
				u32 dest_id, 
				u8 hopcount,
				u32 offset,
				int len,
				u32 paddr,
				u16 type)
{
	unsigned int count;
	unsigned int status = 0;
	unsigned int res    = 0;

	mutex_lock(&_tci648x_rio.lsu_lock);

	/* LSU 1 Reg 0 - MSB of RapidIO address */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG0, 0);

	/* LSU 1 Reg 1 - LSB of destination */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG1, offset);

	/* LSU 1 Reg 2 - source address */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG2, paddr);
	
	/* LSU 1 Reg 3 - byte count */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG3, len);
	
	/* LSU 1 Reg 4 - */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG4,
		       ((index_to_port(index) << 30)
			| (LSU_PRIO << 28)
			| ((mport->sys_size) ? (1 << 24) : 0)
			| ((u32) dest_id << 8)));

	/* LSU 1 Reg 5 */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG5,
		       ((hopcount & 0xff) << 8) | (type & 0xff));
	
	/* Wait for it to complete */
   	count = 0;
	while(1) {
		/* check if LSU has finished and status */
		status = DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_LSU1_REG6);
		if ((status & TCI648X_RIO_LSU_BUSY_MASK) == 0)
			break;
		count++;
		if (count >= TCI648X_RIO_TIMEOUT) {
			res = -EIO; 
			break;
		}
		__delay(100);
	}
	
	mutex_unlock(&_tci648x_rio.lsu_lock);
	
	switch (status & TCI648X_RIO_LSU_CC_MASK) {
	case TCI648X_RIO_LSU_CC_TIMEOUT:
	case TCI648X_RIO_LSU_CC_XOFF:
	case TCI648X_RIO_LSU_CC_ERROR:
	case TCI648X_RIO_LSU_CC_INVALID:
	case TCI648X_RIO_LSU_CC_DMA:
		return -EIO;
	case TCI648X_RIO_LSU_CC_RETRY:
		return -EBUSY;
	case TCI648X_RIO_LSU_CC_CREDIT:
		return -EAGAIN;
	default:
	}

	return 0;
}

/*------------------------- Configuration space mngt  ----------------------*/

/**
 * tci648x_local_config_read - Generate a TCI648x local config space read
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be read into
 *
 * Generates a TCI648x local configuration space read. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int tci648x_local_config_read(struct rio_mport *mport, 
				     int index, u32 offset, int len, u32 * data)
{
	*data = DEVICE_REG32_R(rio_conf_regs + offset);

	DPRINTK("index %d offset 0x%x data 0x%x\n", index, offset, *data);

	return 0;
}

/**
 * tci648x_local_config_write - Generate a TCI648x local config space write
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be written
 *
 * Generates a TCI648x local configuration space write. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int tci648x_local_config_write(struct rio_mport *mport, 
				      int index, u32 offset, int len, u32 data)
{
	DPRINTK("index %d offset 0x%x data 0x%x\n", index, offset, data);

	DEVICE_REG32_W(rio_conf_regs + offset, data);

	return 0;
}

/**
 * tci648x_rio_config_read - Generate a TCI648x read maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Location to be read into
 *
 * Generates a TCI648x read maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int
tci648x_rio_config_read(struct rio_mport *mport, int index, u16 destid,
			u8 hopcount, u32 offset, int len, u32* val)
{
	u32* tbuf;
	int res;

	tbuf = (u32*) kzalloc(L1_CACHE_ALIGN(len), GFP_KERNEL);
	if (!tbuf)
		return -ENOMEM;

	res = maint_request(mport, index, destid, hopcount, offset, len,
			    virt_to_phys(tbuf),
			    TCI648X_RIO_PACKET_TYPE_MAINT_R);

#ifdef CONFIG_TMS320C6X_CACHES_ON
	L2_cache_block_invalidate((u32) tbuf,
				  (u32) tbuf + L1_CACHE_ALIGN(len));
#endif

	/* Taking care of byteswap */
	switch (len) {
        case 1:
	        *val = *((u8*) tbuf);
                break;
        case 2:
	        *val = ntohs(*((u16*) tbuf));
                break;
        default:
                *val = ntohl(*((u32*) tbuf));
                break;
        }

	kfree(tbuf);

	DPRINTK("index %d destid %d hopcount %d offset 0x%x len %d val 0x%x\n",
		index, destid, hopcount, offset, len, *val);

	return res;
}

/**
 * tci648x__rio_config_write - Generate a TCI648x write maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Value to be written
 *
 * Generates an TCI648x write maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int
tci648x_rio_config_write(struct rio_mport *mport, int index, u16 destid, u8 hopcount,
			 u32 offset, int len, u32 val)
{
	u32* tbuf;
	int res;

	DPRINTK("index %d destid %d hopcount %d offset 0x%x len %d val 0x%x\n",
		index, destid, hopcount, offset, len, val);

	tbuf = (u32*) kzalloc(L1_CACHE_ALIGN(len), GFP_KERNEL);
	if (!tbuf)
		return -ENOMEM;

	/* Taking care of byteswap */
	switch (len) {
        case 1:
                *tbuf = ((u8) val);
                break;
        case 2:
                *tbuf = htons((u16) val);
                break;
        default:
                *tbuf = htonl((u32) val);
                break;
        }

#ifdef CONFIG_TMS320C6X_CACHES_ON
	L2_cache_block_writeback((u32) tbuf,
				 (u32) tbuf + L1_CACHE_ALIGN(len));
#endif

	res = maint_request(mport, index, destid, hopcount, offset, len,
			    virt_to_phys(tbuf),
			    TCI648X_RIO_PACKET_TYPE_MAINT_W);

	kfree(tbuf);

	return res;
}

/*------------------------------- Port-Write management --------------------------*/
/**
 * tci648x_rio_pw_enable - enable/disable port-write interface init
 * @mport: Master port implementing the port write unit
 * @enable: 1=enable; 0=disable port-write message handling
 */
static int tci648x_rio_pwenable(struct rio_mport *mport, int enable)
{
	u32 reg;

	/* Enable/Disable port-write-in interrupt */
	reg = DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_MODE);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_MODE,
		       (reg & ~(1 << 1)) | TCI648X_RIO_SP_IP_MODE_DEFAULT | ((enable & 1) << 1));

	return 0;
}

static void tci648x_rio_pw_dpc(struct work_struct *work)
{
	struct tci648x_rio_data *p_rio = container_of(work, struct tci648x_rio_data, pw_work);
	unsigned long            flags;
	u32                      msg_buffer[RIO_PW_MSG_SIZE/sizeof(u32)];

	/*
	 * Process port-write messages
	 */
	spin_lock_irqsave(&p_rio->pw_fifo_lock, flags);
	while (kfifo_out(&p_rio->pw_fifo, 
			 (unsigned char *) msg_buffer,
			 RIO_PW_MSG_SIZE)) {

		/* Process one message */
		spin_unlock_irqrestore(&p_rio->pw_fifo_lock, flags);

#ifdef TCI648X_RIO_DEBUG_PW

		{
		u32 i;
		printk("%s : Port-Write Message:", __func__);
		for (i = 0; i < RIO_PW_MSG_SIZE/sizeof(u32); i++) {
			if ((i%4) == 0)
				printk("\n0x%02x: 0x%08x", i*4,
					msg_buffer[i]);
			else
				printk(" 0x%08x", msg_buffer[i]);
		}
		printk("\n");
		}

#endif /* TCI648X_RIO_DEBUG_PW */

		/* Pass the port-write message to RIO core for processing */
		rio_inb_pwrite_handler((union rio_pw_msg *) msg_buffer);
		spin_lock_irqsave(&p_rio->pw_fifo_lock, flags);
	}
	spin_unlock_irqrestore(&p_rio->pw_fifo_lock, flags);
}


/**
 *  tci648x_rio_port_write_handler - TCI648x port write interrupt handler
 *
 * Handles port write interrupts. Parses a list of registered
 * port write event handlers and executes a matching event handler.
 */
static void tci648x_rio_port_write_handler(struct tci648x_rio_data *p_rio)
{
	u32        reg;
	int        pw;

	/* Check that we have a port-write-in case */
	pw = DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_MODE) & (1 << 0);

	/* Schedule deferred processing if PW was received */
	if (pw) {
		/* 
		 * Retrieve PW message
		 */
		p_rio->port_write_msg.msg.em.comptag =
			DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_PW_IN_CAPT0);
		p_rio->port_write_msg.msg.em.errdetect =
			DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_PW_IN_CAPT1);
		p_rio->port_write_msg.msg.em.is_port =
			DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_PW_IN_CAPT2);
		p_rio->port_write_msg.msg.em.ltlerrdet =
			DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_PW_IN_CAPT3);

		/* 
		 * Save PW message (if there is room in FIFO), otherwise discard it.
		 */
		if (kfifo_avail(&p_rio->pw_fifo) >= RIO_PW_MSG_SIZE) {
			p_rio->port_write_msg.msg_count++;
			kfifo_in(&p_rio->pw_fifo,
				 (void const *) &p_rio->port_write_msg.msg,
				 RIO_PW_MSG_SIZE);
		} else {
			p_rio->port_write_msg.discard_count++;
			printk("RIO: ISR Discarded Port-Write Msg(s) (%d)\n",
				 p_rio->port_write_msg.discard_count);
		}
		schedule_work(&p_rio->pw_work);
	}

	/* Acknowledge port-write-in */
	reg = DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_MODE);
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_SP_IP_MODE,
		       reg | TCI648X_RIO_SP_IP_MODE_DEFAULT | (1 << 0));

	return;
}

/**
 * tci648x_rio_port_write_init - TCI648x port write interface init
 * @mport: Master port implementing the port write unit
 *
 * Initializes port write unit hardware and buffer
 * ring. Called from tci648x_rio_setup(). Returns %0 on success
 * or %-ENOMEM on failure.
 */
static int tci648x_rio_port_write_init(struct tci648x_rio_data *p_rio)
{
	int i;

	/* Following configurations require a disabled port write controller */
	tci648x_rio_pwenable(NULL, 0);

	/* Clear port-write-in capture registers */
	for (i = 0; i < 4; i++)
		DEVICE_REG32_W(TCI648X_RIO_REG_BASE
			       + (TCI648X_RIO_SP_IP_PW_IN_CAPT0 + (i << 2)),
			       0x00000000);

	INIT_WORK(&p_rio->pw_work, tci648x_rio_pw_dpc);
	spin_lock_init(&p_rio->pw_fifo_lock);
	if (kfifo_alloc(&p_rio->pw_fifo, RIO_PW_MSG_SIZE * 32, GFP_KERNEL)) {
		printk("RIO: FIFO allocation failed\n");
		return -ENOMEM;
	}
	return 0;
}

/*----------------------------- Message passing management  ----------------------*/

/**
 * tci648x_rio_map_mbox - Map a mailbox to a given queue
 * @mbox: mailbox to map
 * @queue: associated queue number
 *
 * Returns %0 on success or %-ENOMEM on failure.
 */
static int tci648x_rio_map_mbox(int mbox, int queue, int size)
{
	struct tci648x_rio_msg_ring *rx_ring = &msg_rx_ring[queue];
	u32 mapping_entry_low;
	u32 mapping_entry_high;
	int res;

	/*
	 * Map the multi-segment mailbox to the corresponding Rx queue
	 */
	mapping_entry_low = ((mbox & 0x3) << 16)
 		| (0x3f000000);    /* Mailbox (only mbox) and letter (all) mask */

	mapping_entry_high = TCI648X_RIO_MAP_FLAG_SEGMENT /* multi-segment messaging*/
		| TCI648X_RIO_MAP_FLAG_PROMISC            /* promiscuous (don't care about source id) */
		| ((queue & 0xf) << 2);                   /* queue number */

	if (size)
		mapping_entry_high |= TCI648X_RIO_MAP_FLAG_TT_16; /* tt */
	
	/* Allocate look-up table entries for mbox to queue mapping */
	res = allocate_resource(&_tci648x_rio_rxu_map_res,
				&rx_ring->map_res,
				16,
				TCI648X_RIO_REG_BASE + TCI648X_RIO_RXU_MAP_START,
				TCI648X_RIO_REG_BASE + TCI648X_RIO_RXU_MAP_END,
				8,
				NULL,
				NULL);
	if (res)
		return -ENOMEM;

	DPRINTK("Using RXU map entries 0x%x to 0x%x\n", 
		rx_ring->map_res.start,
		rx_ring->map_res.end);

	DEVICE_REG32_W(rx_ring->map_res.start,     mapping_entry_low);
	DEVICE_REG32_W(rx_ring->map_res.start + 4, mapping_entry_high);

        /*
	 *  The RapidIO peripheral looks at the incoming RapidIO msgs
	 *  and if there is only one segment (the whole msg fits into one
	 *  RapidIO msg), the peripheral uses the single segment mapping
	 *  table. Therefore we need to map the single-segment mailbox too.
	 *  The same Rx CPPI Queue is used (as for the multi-segment
	 *  mailbox).
	 */
	mapping_entry_high &= ~TCI648X_RIO_MAP_FLAG_SEGMENT;
	
	DEVICE_REG32_W(rx_ring->map_res.start + 8,  mapping_entry_low);
	DEVICE_REG32_W(rx_ring->map_res.start + 12, mapping_entry_high);

	return 0;
}

/**
 * rio_open_inb_mbox - Initialize TCI648x inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the inbound mailbox ring
 *
 * Initializes buffer ring, request the inbound message interrupt,
 * and enables the inbound message unit. Returns %0 on success
 * and %-EINVAL or %-ENOMEM on failure.
 */
int rio_open_inb_mbox(struct rio_mport *mport, void *dev_id, int mbox, int entries) 
{
	int                          i;
	int                          res;
	u32                          queue   = mport->id + TCI648X_RIO_MSG_RX_QUEUE_FIRST;
	struct tci648x_rio_msg_ring *rx_ring = &msg_rx_ring[queue];
	struct rio_msg_desc         *desc;

	DPRINTK("mport = 0x%x, dev_id = 0x%x, mbox = %d, entries = %d\n",
		mport, dev_id, mbox, entries);

	/* Check if the port is already registered in this queue */
	if (rx_ring->port == mport)
		return 0;

	if ((entries < TCI648X_RIO_MIN_RX_RING_SIZE) ||
	    (entries > TCI648X_RIO_MAX_RX_RING_SIZE) || (!is_power_of_2(entries))) {
		return -EINVAL;
	}

	/* Allocate descriptor range */
	res = allocate_resource(&_tci648x_rio_desc_res,
				&rx_ring->desc_res,
				entries * sizeof(struct rio_msg_desc),
				TCI648X_RIO_DESC_BASE,
				TCI648X_RIO_DESC_BASE + TCI648X_RIO_DESC_SIZE,
				sizeof(struct rio_msg_desc),
				NULL,
				NULL);
	if (res)
		return -ENOMEM;
				
	desc = (struct rio_msg_desc *) rx_ring->desc_res.start;

	DPRINTK("Using descriptor range 0x%x 0x%x\n", 
		rx_ring->desc_res.start,
		rx_ring->desc_res.end);

	/* Initialize client buffer ring */
	rx_ring->dev_id         = dev_id;
	rx_ring->entries        = entries;
	rx_ring->mbox           = mbox;
	rx_ring->port           = mport;
	rx_ring->slot           = 0;
	rx_ring->count          = 0;
	rx_ring->full           = 0;
	rx_ring->error          = 0;
	rx_ring->running        = 0;
	rx_ring->queue          = queue;
	rx_ring->payload        = (TCI648X_RIO_MSG_BUFFER_SIZE >> 3) & 0x1ff;  
	rx_ring->desc_base      = desc;
	rx_ring->dirty          = desc;
	rx_ring->head           = desc;

	for (i = 0; i < entries; i ++) {
		desc->next  = desc + 1;
		desc->pbuff = NULL;

		if (mport->sys_size)
			desc->opt1 = TCI648X_RIO_DESC_FLAG_TT_16;
		else
			desc->opt1 = 0;

		desc->opt2  = 0; /* not ready */
		desc++;
	}

	/* Terminate the queue */
	(desc - 1)->next = NULL; 

	/* Set the mailbox to queue mapping */
	res = tci648x_rio_map_mbox(mbox, queue, mport->sys_size);
	if (res)
		return res;

	/* Start the Rx queue */
	rx_ring->running = 1;

	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_RXDMA_HDP0 + (queue << 2),
		       virt_to_phys(rx_ring->desc_base));

	return 0;
}

/**
 * rio_close_inb_mbox - Shut down TCI648x inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, stop queues and free all resources
 */
void rio_close_inb_mbox(struct rio_mport *mport, int mbox) 
{
	u32 queue = mport->id + TCI648X_RIO_MSG_RX_QUEUE_FIRST;

	if ((msg_rx_ring[queue].mbox == mbox) && 
	    (msg_rx_ring[queue].port == mport)) {

		int res;
		int tmp;

		/* Teardown the Rx queue */
		DPRINTK("Starting Rx teardown for queue %d\n", queue);

		msg_rx_ring[queue].running = 0;
	
		DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_RX_QUEUE_TEARDOWN,
			       1 << queue);

		if (DEVICE_REG32_R(TCI648X_RIO_REG_BASE +  TCI648X_RIO_TXDMA_HDP0 
				   + (queue << 2)) != NULL) {

			for (tmp = 0; tmp != 0xfffffffc;
			     tmp = DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_RXDMA_CP0
						  + (queue << 2)))
				DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_RXDMA_CP0
					       + (queue << 2), tmp);
		}

		DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_RX_CPPI_ICCR,
			       1 << queue);

		msg_rx_ring[queue].port = NULL;

		/* Release resources */
		res = release_resource(&msg_rx_ring[queue].desc_res);
		if (res)
			printk("release of resource 0x%x failed", msg_rx_ring[queue].desc_res.start);

		res = release_resource(&msg_rx_ring[queue].map_res);
		if (res)
			printk("release of resource 0x%x failed", msg_rx_ring[queue].map_res.start);

	}
	return;
}

/**
 * rio_open_outb_mbox - Initialize TCI648x outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the outbound mailbox ring
 *
 * Initializes buffer ring, request the outbound message interrupt,
 * and enables the outbound message unit. Returns %0 on success and
 * %-EINVAL or %-ENOMEM on failure.
 */
int rio_open_outb_mbox(struct rio_mport *mport, void *dev_id, int mbox, int entries)
{
	int                          i;
	int                          res;
	u32                          queue   = mport->id + TCI648X_RIO_MSG_TX_QUEUE_FIRST;
	struct tci648x_rio_msg_ring *tx_ring = &msg_tx_ring[queue];
	struct rio_msg_desc         *desc;

	DPRINTK("mport = 0x%x, dev_id = 0x%x, mbox = %d, entries = %d\n",
		mport, dev_id, mbox, entries);

	/* Check if already initialized, multiport/mbox not yet supported */
	if (tx_ring->port == mport)
		return 0;

	if ((entries < TCI648X_RIO_MIN_TX_RING_SIZE) ||
	    (entries > TCI648X_RIO_MAX_TX_RING_SIZE) || (!is_power_of_2(entries))) {
		return -EINVAL;
	}

	/* Allocate descriptor range */
	res = allocate_resource(&_tci648x_rio_desc_res,
				&tx_ring->desc_res,
				entries * sizeof(struct rio_msg_desc),
				TCI648X_RIO_DESC_BASE,
				TCI648X_RIO_DESC_BASE + TCI648X_RIO_DESC_SIZE,
				sizeof(struct rio_msg_desc),
				NULL,
				NULL);
	if (res)
		return -ENOMEM;
			      
	desc = (struct rio_msg_desc *) tx_ring->desc_res.start;

	DPRINTK("Using descriptor range 0x%x 0x%x\n", 
		tx_ring->desc_res.start,
		tx_ring->desc_res.end);

	/* Initialize client buffer ring */
	tx_ring->dev_id         = dev_id;
	tx_ring->entries        = entries;
	tx_ring->mbox           = mbox;
	tx_ring->port           = mport;
	tx_ring->slot           = 0;
	tx_ring->count          = 0;
	tx_ring->full           = 0;
	tx_ring->error          = 0;
	tx_ring->running        = 0;
	tx_ring->queue          = queue;
	tx_ring->payload        = (TCI648X_RIO_MSG_BUFFER_SIZE >> 3) & 0x1ff;
	tx_ring->desc_base      = desc;
	tx_ring->dirty          = desc;
	tx_ring->head           = desc;

	for (i = 0; i < entries; i ++) {
		desc->next  = NULL;
		desc->pbuff = NULL;
		desc->opt1  = 0;
		desc->opt2  = 0;
		desc++;
	}

	tx_ring->running = 1;

	return 0;
}

/**
 * rio_close_outb_mbox - Shut down TCI648x outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, stop queues and free all resources
 */
void rio_close_outb_mbox(struct rio_mport *mport, int mbox)
{
	u32 queue = mport->id + TCI648X_RIO_MSG_TX_QUEUE_FIRST;

	if ((msg_tx_ring[queue].mbox == mbox) && 
	    (msg_tx_ring[queue].port == mport)) {

		int res;
		int tmp;

		/* Teardown the Tx queue */
		DPRINTK("Starting Tx teardown for queue %d\n", queue);

		msg_tx_ring[queue].running = 0;

		DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_TX_QUEUE_TEARDOWN,
			       1 << queue);

		if (DEVICE_REG32_R(TCI648X_RIO_REG_BASE +  TCI648X_RIO_TXDMA_HDP0 
				   + (queue << 2)) != NULL) {

			for (tmp = 0; tmp != 0xfffffffc;
			     tmp = DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_TXDMA_CP0
						  + (queue << 2)))
				DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_TXDMA_CP0
					       + (queue << 2), tmp);
		}

		DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_TX_CPPI_ICCR,
			       1 << queue);

		msg_tx_ring[queue].port = NULL;
			
		/* Release resources */
		res = release_resource(&msg_tx_ring[queue].desc_res);
		if (res)
			printk("release of resource 0x%x failed", msg_tx_ring[queue].desc_res.start);
	}
	return;
}

/**
 * rio_hw_add_outb_message - Add a message to the TCI648x outbound message queue
 * @mport: Master port with outbound message queue
 * @rdev: Target of outbound message
 * @mbox: Outbound mailbox
 * @buffer: Message to add to outbound queue
 * @len: Length of message
 *
 * Adds the @buffer message to the TCI648x outbound message queue. Returns
 * %0 on success or %-EBUSY on failure.
 */
int rio_hw_add_outb_message(struct rio_mport *mport, struct rio_dev *rdev, 
			    int mbox, void *buffer, const size_t len)
{
	unsigned long                flags;
	int                          res     = 0;
	u32                          queue   = mport->id + TCI648X_RIO_MSG_TX_QUEUE_FIRST;
	struct tci648x_rio_msg_ring *tx_ring = &msg_tx_ring[queue];
	struct rio_msg_desc         *desc    = get_cur_desc(tx_ring);

	if (len == 0)
		return -EINVAL;

	if (tx_ring->port == NULL)
		return -EINVAL;

	if (tx_ring->full == 1)
		return -EBUSY;

#ifdef CONFIG_TMS320C6X_CACHES_ON
	/* No coherency is assumed between CPPI and L2 cache */
	L2_cache_block_writeback((u32) buffer,
				 (u32) buffer + len);
#endif

	spin_lock_irqsave(&tx_ring->lock, flags);

	desc->pbuff = virt_to_phys(buffer);            /* buffer */
	desc->next  = NULL;
	desc->opt1  = (mbox & (TCI648X_MAX_MBOX - 1))  /* mbox */
		| (TCI648X_RIO_MSG_SSIZE << 6)         /* ssize (32 dword) */
		| (mport->id << 10)                    /* port_id */
		| ((u32) rdev->destid << 16);          /* dest_id*/
	
	if (rdev->net->hport->sys_size)
		desc->opt1 |= TCI648X_RIO_DESC_FLAG_TT_16;     /* tt */

	desc->opt2 = (((len + 7) >> 3) & 0xff )        /* message_length */
		| TCI648X_RIO_DESC_FLAG_OWNER          /* ownership set to port */
		| TCI648X_RIO_DESC_FLAG_EOP
		| TCI648X_RIO_DESC_FLAG_SOP;           /* eop, sop */
	     
	/* Get the previous element of the ring if we are not the head */
	if (desc != tx_ring->head) {
		struct rio_msg_desc *prev_desc = get_prev_desc(desc, tx_ring);
		
		/* Link the buffer to the previous one in the list */
		prev_desc->next = (struct rio_msg_desc *) virt_to_phys(desc);
	}

	/* Go to next descriptor */
	if (++tx_ring->slot == tx_ring->entries)
		tx_ring->slot = 0;

	tx_ring->count++;

	/* 
	 * If we are the new head and there is no descriptor to acknowledge, start 
         * the new head.
	 */
	if ((desc == tx_ring->head) && (tx_ring->count == 1)) {
		
		DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_TXDMA_HDP0
			       + (tx_ring->queue << 2),
			       virt_to_phys(tx_ring->head));

		/* Set the new head to next desc */
		tx_ring->head = get_cur_desc(&msg_tx_ring[queue]);  
	}

	/* Check ring oll over: do not reach the not yet acknowledged packets */
	if (tx_ring->count == tx_ring->entries) {

		tx_ring->full = 1;

		DPRINTK("MP TX queue full count = %d slot = 0x%x head = 0x%x\n",
			tx_ring->count, tx_ring->slot, tx_ring->head);
	}

	spin_unlock_irqrestore(&tx_ring->lock, flags);

	return res;
}

/*
 * CPPI interrupt handler for Tx
 */
static void cppi_tx_handler(u32 queue)
{
	struct rio_msg_desc         *desc_next = NULL;
	unsigned long                flags;
	struct tci648x_rio_msg_ring *tx_ring   = &msg_tx_ring[queue];
	struct rio_mport            *port      = tx_ring->port;
	struct rio_msg_desc         *desc      = tx_ring->dirty;

	if ((tx_ring->port == NULL) || (!tx_ring->running)) {
		/*
		 * This queue is disabled or within the teardown procedure so 
		 * acknowledge the CP to avoid spurious interrupts 
		 */
		u32 ack = DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_TXDMA_CP0
					 + (queue << 2));
		DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_TXDMA_CP0
			       + (queue << 2), ack);
		return;
	}

	/* Check spurious interrupt (nothing to free) */
	if (tx_ring->count == 0) {
		DPRINTK("No desc to free\n");
		return;
	}

	for(;;) {
		u32 cc;

		ASSERT(tx_ring->count > 0);

	        /* Cleanup the desc */
		atomic_sub(1, &tx_ring->count);

		/* Check teardown case */
		if (desc->opt2 & TCI648X_RIO_DESC_FLAG_TEARDOWN)
			return;

		/* Check error cases */
		cc = (desc->opt2 & TCI648X_RIO_DESC_FLAG_CC) >> 9;
		if (cc) {
			DPRINTK("Tx ack error cc = 0x%x, desc = 0x%x\n", cc, desc);
			desc->opt2 & ~TCI648X_RIO_DESC_FLAG_CC;
			tx_ring->error++;
		}

		/* Move to next desc */
		desc_next = desc + 1;
		if (desc_next >= tx_ring->desc_base + tx_ring->entries)
			desc_next = tx_ring->desc_base;

		desc->pbuff = NULL;
		desc->next  = NULL;

		/* Check if we reach EOQ */
		if (desc->opt2 & TCI648X_RIO_DESC_FLAG_EOQ) {

			tx_ring->head = desc_next;

			/* If there are some waiting Tx packets, start the new queue */
			if (tx_ring->count > 0) {

				/* Start the new queue */
				DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_TXDMA_HDP0
					       + (queue << 2),
					       virt_to_phys(tx_ring->head));
			}

			break;
		}

		/* Exit if next desc is still owned by the port */
		if (desc_next->opt2 & TCI648X_RIO_DESC_FLAG_OWNER)
			break;

		desc = desc_next;
	}

	/* Client is in charge of freeing the associated buffers */
	port->outb_msg[tx_ring->mbox].mcback(port,
					     tx_ring->dev_id,
					     tx_ring->mbox,
					     desc_next - tx_ring->desc_base);
	
	/* Since we have freed up at least a buffer, the ring is no longer full */
	if (tx_ring->full)
		tx_ring->full = 0;

	/* Update dirty tx to next one */
	tx_ring->dirty = desc_next;

	/* ACK the last good desc */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_TXDMA_CP0
		       + (queue << 2), desc);
}

/**
 * rio_hw_add_inb_buffer - Add buffer to the TCI648x inbound message queue
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 * @buf: Buffer to add to inbound queue
 *
 * Adds the @buf buffer to the TCI648x inbound message queue. Returns
 * %0 on success or %-EINVAL on failure.
 */
int rio_hw_add_inb_buffer(struct rio_mport *mport, int mbox, void *buffer)
{
	u32                          queue     = mport->id + TCI648X_RIO_MSG_RX_QUEUE_FIRST;
	struct tci648x_rio_msg_ring *rx_ring   = &msg_rx_ring[queue];
	struct rio_msg_desc         *dirty     = rx_ring->dirty;
	struct rio_msg_desc         *prev_desc = get_prev_desc(dirty, rx_ring);
	u32                          pkt_flags = dirty->opt2;

	ASSERT(dirty->pbuff == NULL);

	/* Descriptor is now available */
	dirty->pbuff    = virt_to_phys(buffer);
	dirty->next     = NULL;
	
	dirty->opt1  = 0;

	if (mport->sys_size)
		dirty->opt1 |= TCI648X_RIO_DESC_FLAG_TT_16;

	dirty->opt2  = rx_ring->payload
		| TCI648X_RIO_DESC_FLAG_OWNER
		| TCI648X_RIO_DESC_FLAG_EOP
		| TCI648X_RIO_DESC_FLAG_SOP;

	/* Increase queue with this new desc */
	prev_desc->next = dirty;

	/* Update RX dirty */
	if (++rx_ring->dirty >= (rx_ring->desc_base + rx_ring->entries))
		rx_ring->dirty = rx_ring->desc_base;

	/* Test if the queue must be restarted */
	if (pkt_flags & TCI648X_RIO_DESC_FLAG_EOQ) {

		ASSERT(DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_RXDMA_HDP0
				      + (queue << 2)) == NULL);
		
		/* Start the Rx queue with the next desc */
		DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_RXDMA_HDP0
			       + (queue << 2),
			       virt_to_phys(rx_ring->dirty));
		
		rx_ring->head = rx_ring->dirty;
	}

	return 0;
}

/**
 * rio_hw_get_inb_message - Fetch inbound message from the TCI648x message unit
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 *
 * Gets the next available inbound message from the inbound message queue.
 * A pointer to the message is returned on success or NULL on failure.
 */
void *rio_hw_get_inb_message(struct rio_mport *mport, int mbox)
{
	u32                          queue     = mport->id + TCI648X_RIO_MSG_RX_QUEUE_FIRST;
	struct tci648x_rio_msg_ring *rx_ring   = &msg_rx_ring[queue];
	int                          slot      = rx_ring->slot;
	struct rio_msg_desc         *desc      = get_cur_desc(rx_ring);
	u32                          pkt_flags = desc->opt2;
	u16                          pkt_len   = ((pkt_flags & 0xff) ?
						  (pkt_flags & 0xff) << 3 :
						  TCI648X_RIO_MSG_MAX_BUFFER_SIZE);
	u32                          cc;
	void                        *buff      = phys_to_virt(desc->pbuff);
	struct rio_msg_desc         *prev_desc = get_prev_desc(desc, rx_ring);

	ASSERT(buff != NULL);

	/* Check teardown case */
	if (desc->opt2 & TCI648X_RIO_DESC_FLAG_TEARDOWN)
		return NULL;

	/* Test if we finished (if the current packet is owned by the ports) */
	if (pkt_flags & TCI648X_RIO_DESC_FLAG_OWNER)
		return NULL;

	/* Test end of queue */
	if ((desc != rx_ring->head) &&
	    (prev_desc->opt2 & TCI648X_RIO_DESC_FLAG_EOQ)) {
		ASSERT(prev_desc->next == NULL);
		return NULL;
	}

	/* Rx error case */
	cc = (desc->opt2 & TCI648X_RIO_DESC_FLAG_CC) >> 9;
	if (cc) {	     
		DPRINTK("Rx error cc = 0x%x, desc = 0x%x\n", cc, desc);
		desc->opt2 & ~TCI648X_RIO_DESC_FLAG_CC;
		rx_ring->error++;
	}
	
#ifdef CONFIG_TMS320C6X_CACHES_ON
	/* No coherency is assumed between CPPI and L2 cache */
	L2_cache_block_invalidate((u32) buff,
				  (u32) buff + pkt_len);
#endif

	desc->pbuff = NULL;

	/* Loop in the ring */
	if (++desc == (rx_ring->desc_base + rx_ring->entries))
		desc = rx_ring->desc_base;

	rx_ring->slot = desc - rx_ring->desc_base;

	return buff;
}

/*
 * CPPI interrupt handler for Rx
 */
static void cppi_rx_handler(u32 queue)
{
	struct tci648x_rio_msg_ring *rx_ring   = &msg_rx_ring[queue];
	struct rio_msg_desc         *desc      = get_cur_desc(rx_ring);
	struct rio_msg_desc         *prev_desc = get_prev_desc(desc, rx_ring);

	if ((rx_ring->port == NULL) || (!rx_ring->running)) {
		/*
		 * This queue is disabled or within the teardown procedure so 
		 * acknowledge the CP to avoid spurious interrupts 
		 */
		u32 ack = DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_RXDMA_CP0
					 + (queue << 2));
		DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_RXDMA_CP0
			       + (queue << 2), ack);
		return;
	}

retry:
	/* Client callback (slot parameter is not used) */
	rx_ring->port->inb_msg[rx_ring->mbox].mcback(rx_ring->port,
						     rx_ring->dev_id,
						     rx_ring->mbox,
						     rx_ring->slot);
	
	/* ACK the last managed desc */
	DEVICE_REG32_W(TCI648X_RIO_REG_BASE + TCI648X_RIO_RXDMA_CP0
		       + (queue << 2), prev_desc);

	/*
	 *  When writing the CP, there is a small race condition
	 *  with the setting of the ICSR register in the peripheral.
	 *  A simple and valid work-around is to re-check the
	 *  ownership bit.  If it is still the ports, do nothing.
	 *  If it is the hosts, process the packet as normal.
	 */
	if ((desc->opt2 & TCI648X_RIO_DESC_FLAG_OWNER == 0) ||
	    (desc->opt2 & TCI648X_RIO_DESC_FLAG_EOQ))
		goto retry;
}

EXPORT_SYMBOL_GPL(rio_hw_add_outb_message);
EXPORT_SYMBOL_GPL(rio_hw_add_inb_buffer);
EXPORT_SYMBOL_GPL(rio_hw_get_inb_message);

/*-------------------------- Main Linux driver functions -----------------------*/

static char *cmdline_hdid  = NULL;
static char *cmdline_ports = NULL;
static char *cmdline_init  = NULL;
static char *cmdline_mode  = NULL;

static int tci648x_rio_get_hdid(int index, int default_id, int size)
{
	int id;

        if (!cmdline_hdid) 
                id = default_id;
	else {
		id = simple_strtol(cmdline_hdid, NULL, 0);
	}

	if (id < 0)
		/* Read the default one */
		id = TCI648X_RIO_GET_DID(
			DEVICE_REG32_R(TCI648X_RIO_REG_BASE + TCI648X_RIO_BASE_ID),
			size);

	return id;
}

static int tci648x_rio_get_ports(int index, int default_ports)
{
	int ports = 0;
	int port;

        if (!cmdline_ports) 
                ports = default_ports;
	else {
		while(get_option(&cmdline_ports, &port))
			ports |= (1 << port);
	}
	return ports;
}

static int tci648x_rio_get_init(int index, int default_init)
{
	int init = 0;

        if (!cmdline_init) 
                init = default_init;
	else {
		if (strcmp(cmdline_init, "enum") == 0) 
			init = RIO_DO_ENUMERATION;
		else if (strcmp(cmdline_init, "discov") == 0)
			init = RIO_DO_DISCOVERY;
		else if (strcmp(cmdline_init, "discov_wait") == 0)
			init = RIO_DO_DISCOVERY_WAIT;
		else 
			init = default_init;
	}
	return init;
}

static int tci648x_rio_get_mode(int index, int default_mode)
{
	int mode = 0;

        if ((!cmdline_mode) || (!get_option(&cmdline_mode, &mode)))
                mode = default_mode;

	return mode;
}

static int tci648x_rio_get_cmdline_hdid(char *s)
{
        if (!s)
                return 0;
        cmdline_hdid = s;
        return 1;
}

__setup("riohdid=", tci648x_rio_get_cmdline_hdid);

static int tci648x_rio_get_cmdline_ports(char *s)
{
        if (!s)
                return 0;
        cmdline_ports = s;
        return 1;
}

__setup("rioports=", tci648x_rio_get_cmdline_ports);

static int tci648x_rio_get_cmdline_init(char *s)
{
        if (!s)
                return 0;
        cmdline_init = s;
        return 1;
}
__setup("rioinit=", tci648x_rio_get_cmdline_init);

static int tci648x_rio_get_cmdline_mode(char *s)
{
        if (!s)
                return 0;
        cmdline_mode = s;
        return 1;
}
__setup("riomode=", tci648x_rio_get_cmdline_mode);

#ifdef CONFIG_EDMA3
static int tci648x_rio_get_cmdline_edma_threshold(char *s)
{
        if (!s)
                return 0;

        get_option(&s, &tci648x_edma_threshold);
        return 1;
}
__setup("rioedma-threshold=", tci648x_rio_get_cmdline_edma_threshold);
#endif /* CONFIG_EDMA3 */

/**
 * tci648x_rio_fixup - Architecture specific fixup fonction
 * @mport: RapidIO master port info
 * @rdev: Associated RIO dev structure
 */
static int tci648x_rio_fixup(struct rio_mport *mport, struct rio_dev *rdev)
{
	rdev->dio.base_offset = RAM_MEMORY_START; /* For EVM test purpose */

	/* Add flow control for this device */
	tci648x_add_flow_control(rdev);

	return 0;
}

int tci648x_rio_register_mport(u32 port_id, u32 hostid, u32 init)
{
	struct rio_ops   *ops;
	struct rio_mport *port;
        int               res;

	ops = kzalloc(sizeof(struct rio_ops), GFP_KERNEL);

	ops->lcread       = tci648x_local_config_read;
	ops->lcwrite      = tci648x_local_config_write;
	ops->cread        = tci648x_rio_config_read;
	ops->cwrite       = tci648x_rio_config_write;
	ops->dsend        = tci648x_rio_dbell_send;
	ops->dwait        = tci648x_rio_dbell_wait;
	ops->transfer     = tci648x_rio_dio_transfer;
	ops->fixup        = tci648x_rio_fixup;
	ops->pwenable     = tci648x_rio_pwenable;

	port = kzalloc(sizeof(struct rio_mport), GFP_KERNEL);
	port->id          = port_id;
	port->index       = port_to_index(port_id);
	INIT_LIST_HEAD(&port->dbells);

	/* Make a dummy per port region as ports are not really separated on TCI648x */
	port->iores.start = TCI648X_RIO_REG_BASE + TCI648X_RIO_SP0_ERR_STAT + (port_id << 5);
	port->iores.end   = TCI648X_RIO_REG_BASE + TCI648X_RIO_SP0_CTL + (port_id << 5);
	port->iores.flags = IORESOURCE_MEM;

	rio_init_dbell_res(&port->riores[RIO_DOORBELL_RESOURCE], 0, 0xffff);
	rio_init_mbox_res(&port->riores[RIO_INB_MBOX_RESOURCE], 0, 0);
	rio_init_mbox_res(&port->riores[RIO_OUTB_MBOX_RESOURCE], 0, 0);

	sprintf(port->name, "RIO%d mport", port_id);

	port->ops           = ops;
	port->init          = init;
	port->host_deviceid = hostid;
	port->sys_size      = 1; /* AJ: 16bits */
	port->phy_type      = RIO_PHY_SERIAL;

	rio_register_mport(port);

	return 0;
}

/*
 * Platform configuration setup
 */
static int tci648x_rio_setup_controller(struct platform_device *pdev)
{
     	struct tci648x_rio_board_controller_info *c;
	struct device *dev;
	u32 hostid;
	u32 ports;
	u32 init;
	u32 mode;
	int res = 0;
	int idx = pdev->id - 1;

	dev = get_device(&pdev->dev);
	c   = (struct tci648x_rio_board_controller_info *) dev->platform_data;

	hostid = tci648x_rio_get_hdid(idx, c->id, c->size);
	ports  = tci648x_rio_get_ports(idx, c->ports);
	init   = tci648x_rio_get_init(idx, c->init);
	mode   = tci648x_rio_get_mode(idx, c->mode);

	printk("RIO: register sRIO controller for hostid %d\n", hostid);
	
	/* Hardware set up of the controller */
	tci648x_rio_hw_init(mode, hostid, c->size);
	
	/* Start the controller */
	tci648x_rio_start();
	
	while(ports) {
		int status;
		u32 port = __ffs(ports);
		ports &= ~(1 << port);
		
		/* Initialize this port */
		res = tci648x_rio_port_init(port, mode);
		if (res < 0)
			goto out;
		
		status = tci648x_rio_port_status(port);
		if (status == 0) {
			/* Register this port  */
			res = tci648x_rio_register_mport(port, hostid, init);
			if (res)
				goto out;
			
			DPRINTK("RIO port %d registered\n", port);
		} else
			DPRINTK("RIO port %d not ready\n", port);
	}
out:
	return res;
}

static int __init tci648x_rio_probe(struct platform_device *pdev)	
{
	int res;
	int port;

	/* sRIO main driver hw initialization (global setup, PSC, interrupts) */
	res = tci648x_rio_init();
	if (res < 0)
		return res;

	/* Setup the sRIO controller */
	res = tci648x_rio_setup_controller(pdev);
	if (res < 0)
		return res;

#ifdef CONFIG_RAPIDIO_DEV
	/* Register userspace interface */ 
	res = rio_dev_init();
	if (res < 0)
		return res;
#endif
	/* Enumerate or discover all registered ports */
	rio_init_mports();

	printk(banner);

#ifdef CONFIG_EDMA3
	printk(KERN_INFO "RIO: setting EDMA threshold to 0x%x\n", tci648x_edma_threshold);
#endif

	return 0;
}

static int __exit tci648x_rio_remove(struct platform_device *pdev)
{
	tci648x_rio_release();

#ifdef CONFIG_RAPIDIO_DEV
	rio_dev_exit();
#endif
	put_device(&pdev->dev);
	return 0;
}

static struct platform_driver tci648x_rio_driver  = {
	.driver = {
		.name	= "tci648x-rapidio",
		.owner	= THIS_MODULE,
	},
	.probe	= tci648x_rio_probe,
	.remove = __exit_p(tci648x_rio_remove),
};

static int tci648x_rio_module_init(void)
{
	return platform_driver_register(&tci648x_rio_driver);
}

static void tci648x_rio_module_exit(void)
{
	platform_driver_unregister(&tci648x_rio_driver);
}

subsys_initcall(tci648x_rio_module_init);
module_exit(tci648x_rio_module_exit);

MODULE_AUTHOR("Aurelien Jacquiot");
MODULE_DESCRIPTION("TCI648x RapidIO device driver");
MODULE_LICENSE("GPL");
