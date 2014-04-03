/*
 * Copyright (C) 2010, 2011, 2012 Texas Instruments Incorporated
 * Authors: Aurelien Jacquiot <a-jacquiot@ti.com>
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
#include <linux/kfifo.h>
#include <linux/delay.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>

#include <mach/keystone_qmss.h>
#include <linux/keystone/qmss.h>
#include <linux/keystone/pktdma.h>
#include <linux/keystone/rio.h>

#undef KEYSTONE_RIO_DEBUG
#ifdef KEYSTONE_RIO_DEBUG
#define DPRINTK(fmt, args...) printk(KERN_DEBUG "RIO: [%s] " fmt, __FUNCTION__ , ## args)
#define ASSERT(cond) if (!(cond)) DPRINTK("ASSERT %s FAILURE\n", # cond)
#else
#define ASSERT(cond)
#define DPRINTK(fmt, args...) 
#endif

static char banner[] __initdata = KERN_INFO "KeyStone RapidIO driver v1.1\n";

static u32 rio_regs;
static u32 rio_conf_regs;
static u32 rio_pe_feat;

struct port_write_msg {
	union rio_pw_msg msg;
	u32              msg_count;
	u32              err_count;
	u32              discard_count;
};

/*
 * Main KeyStone RapidIO driver data
 */
struct keystone_rio_data {
	struct completion	lsu_completion;
	struct mutex		lsu_lock;
	wait_queue_head_t	dbell_waitq[KEYSTONE_RIO_DBELL_VALUE_MAX];
	spinlock_t		dbell_i_lock;
	struct port_write_msg	port_write_msg;
	struct work_struct	pw_work;
	struct kfifo		pw_fifo;
	spinlock_t		pw_fifo_lock;
	struct pktdma_rx_cfg   *rx_cfg;
	struct pktdma_tx_cfg   *tx_cfg;
	u32                     mbox_type;        /* type of packet for outbound mailboxes */
	u32                     max_mbox;         /* number of mailboxes */
	u32                     free_queue;       /* queue for free descriptors */
	u32                     rx_queues[KEYSTONE_RIO_MAX_MBOX];      /* receive queues per mailbox */
	u32                     rx_free_queues[KEYSTONE_RIO_MAX_MBOX]; /* pool of free message per mailbox*/
	u32                     tx_queue;         /* transmit queue */
	u32                     tx_cp_queue;      /* transmit completion queue */
	u32                     rx_channel;       /* receive channel */
	u32                     tx_channel;       /* transmit channel */
	u32                     rx_flow;          /* receive flow */
	u32                     rx_irqs;          /* receive irqs */
	u32                     tx_irq;           /* transmit completion irq */
};

static struct keystone_rio_data       _keystone_rio;
static struct keystone_serdes_config *_keystone_serdes_config;

static void dbell_handler(struct keystone_rio_data *p_rio);
static void keystone_rio_port_write_handler(struct keystone_rio_data *p_rio);
static int  keystone_rio_port_write_init(struct keystone_rio_data *p_rio);
static void keystone_rio_interrupt_setup(void);
static void keystone_rio_interrupt_release(void);
static void keystone_rio_mp_exit(int idx);

#define index_to_port(index) (index)
#define port_to_index(port)  (port)

struct keystone_rio_desc_info {
	u32 ps_desc[2];
	u32 mbox;
};

struct keystone_rio_mbox_info {
	struct rio_mport *port;
	u32               id;
	u32               running;
	u32               entries;
	u32               slot;
	void             *dev_id;
	struct resource   map_res;
};

static struct keystone_rio_mbox_info  keystone_rio_rx_mbox[KEYSTONE_RIO_MAX_MBOX];
static struct keystone_rio_mbox_info  keystone_rio_tx_mbox[KEYSTONE_RIO_MAX_MBOX];
static struct resource                keystone_rio_rxu_map_res;
static void __iomem                  *keystone_rio_cdma_base_addr;

/*------------------------- RapidIO hw controller setup ---------------------*/

static inline int keystone_rio_read_hdid(int size)
{
	int id;

	id = KEYSTONE_RIO_GET_DID(DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_BASE_ID),
				  size);
	return id;
}

/**
 * keystone_rio_hw_init - Configure a RapidIO controller
 * @mode: serdes configuration
 * @hostid: device id of the host
 */
static void keystone_rio_hw_init(u32 mode, u16 hostid)
{
	u32 val;
	u32 block;
	u32 port;

	/* Set sRIO out of reset */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PCR, 0x00000011);

	/* Clear BOOT_COMPLETE bit (allowing write) */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PER_SET_CNTL, 0x00000000);

	/* Enable blocks */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_GBL_EN_REG, 1);
	for (block = 0; block <= KEYSTONE_RIO_BLK_NUM; block++)
		DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_BLK_EN_REG(block), 1);

	/* Set control register 1 configuration */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PER_SET_CNTL1, 0x00000000);

	/* Set Control register */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PER_SET_CNTL, 
		       _keystone_serdes_config[mode].cfg_cntl);

	/* Serdes main configuration */
	DEVICE_REG32_W(KEYSTONE_RIO_SERDES_REG_BASE + KEYSTONE_RIO_SERDES_CFG_PLL_REG,
		       _keystone_serdes_config[mode].serdes_cfg_pll);

	/* Per-port SerDes configuration */
	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		DEVICE_REG32_W(KEYSTONE_RIO_SERDES_REG_BASE + KEYSTONE_RIO_SERDES_CFG_RX_REG(port),
			       _keystone_serdes_config[mode].rx_chan_config[port]);
		DEVICE_REG32_W(KEYSTONE_RIO_SERDES_REG_BASE + KEYSTONE_RIO_SERDES_CFG_TX_REG(port),
			       _keystone_serdes_config[mode].tx_chan_config[port]);
	}
	
	/* Check for RIO SerDes PLL lock */
	do {
		val = DEVICE_REG32_R(KEYSTONE_RIO_SERDES_REG_BASE + KEYSTONE_RIO_SERDES_STS_REG);
    	} while ((val & 0x1) != 0x1);

	/* Set prescalar for ip_clk */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PRESCALAR_SRV_CLK, 
		       _keystone_serdes_config[mode].prescalar_srv_clk);

	/* Peripheral-specific configuration and capabilities */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DEV_ID,     KEYSTONE_RIO_DEV_ID_VAL);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DEV_INFO,   KEYSTONE_RIO_DEV_INFO_VAL);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_ASBLY_ID,   0x00000030); /* TI */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_ASBLY_INFO, 0x00000100);

	rio_pe_feat = RIO_PEF_PROCESSOR
		| RIO_PEF_CTLS
		| RIO_PEF_FLOW_CONTROL
		| RIO_PEF_EXT_FEATURES 
		| RIO_PEF_ADDR_34
		| RIO_PEF_STD_RT
		| RIO_PEF_INB_DOORBELL
		| RIO_PEF_INB_MBOX;

	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PE_FEAT, rio_pe_feat);

	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SW_PORT, KEYSTONE_RIO_MAX_PORT << 8);

	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SRC_OP,       
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

	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DEST_OP,
		       RIO_DST_OPS_READ
		       | RIO_DST_OPS_WRITE
		       | RIO_DST_OPS_STREAM_WRITE
		       | RIO_DST_OPS_WRITE_RESPONSE
		       | RIO_DST_OPS_DATA_MSG
		       | RIO_DST_OPS_DOORBELL
		       | RIO_DST_OPS_PORT_WRITE);

	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PE_LL_CTL,
		       RIO_PELL_ADDR_34);

	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_MB_HEAD,   0x10000002);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LCL_CFG_HBAR, 0x00000000);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LCL_CFG_BAR,  0x00520000);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_GEN_CTL,   0xE0000000); /* Enable HOST & MASTER_ENABLE bits */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_LT_CTL,    0x000FFF00);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_RT_CTL,    0x000FFF00);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_RST_CTL,      0x00000001);

	/* Set error detection mode */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_ERR_DET,      0x00000000);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_ERR_EN,       0x00000000);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_ERR_RPT_BH,   0x30000007);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_H_ADDR_CAPT,  0x00000000);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_ADDR_CAPT,    0x00000000);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_ID_CAPT,      0x00000000);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_CTRL_CAPT,    0x00000000);

	/* Force all writes to finish */
	val = DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_CTRL_CAPT);
}

/**
 * keystone_rio_start - Start RapidIO controller
 */
static void keystone_rio_start(void)
{
	u32 val;

	/* Set PEREN bit to enable logical layer data flow */	
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PCR, 0x0000005);

	/* Set BOOT_COMPLETE bit */
	val = DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PER_SET_CNTL);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PER_SET_CNTL, val | 0x01000000);
}

/**
 * keystone_add_flow_control - Add flow control for a given device ID
 */
static int keystone_add_flow_control(struct rio_dev *rdev)
{
	static int free_entry = 0;
	u32        flow_mask;
	/* The offset of the TXU queue we are using */
	u32        qid = _keystone_rio.tx_queue - DEVICE_QM_SRIO_Q;

	if (free_entry == 16)
		return -ENOMEM;

	/* Setup flow control */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_FLOW_CNTL(free_entry),
		       rdev->net->hport->sys_size ? 0x10000 | rdev->destid : rdev->destid);

	DPRINTK("adding flow control entry %d for devid %d\n", free_entry, rdev->destid);

	/* Enable control flow for LSU */
	flow_mask  = DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE
				    + KEYSTONE_RIO_LSU_FLOW_MASKS0);
	flow_mask |= 1 << free_entry;
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_FLOW_MASKS0,
		       flow_mask);

	DPRINTK("LSU flow mask 0x%x\n", flow_mask);

	/* Enable control flow for TXU */
	flow_mask  = DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE
				    + KEYSTONE_RIO_TX_CPPI_FLOW_MASK(qid >> 1));
	flow_mask |= 1 << (free_entry + ((qid & 0x1) << 4));
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE
		       + KEYSTONE_RIO_TX_CPPI_FLOW_MASK(qid >> 1), flow_mask);
	
	DPRINTK("TX queue %d flow mask 0x%x\n", qid, flow_mask);

	free_entry++;
	return 0;
}

/**
 * keystone_rio_port_status - Return if the port is OK or not
 * @port: index of the port
 *
 * Return %0 if the port is ready or %-EIO on failure.
 */
static int keystone_rio_port_status(int port)
{
	unsigned int count, value, portok;

	count  = 0;
	portok = 0;

	if (port >= KEYSTONE_RIO_MAX_PORT)
		return -EINVAL;

	/* Check status */
	while(1) { 
		value =	DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE
				       + KEYSTONE_RIO_SP_ERR_STAT(port));
		
		if ((value & RIO_PORT_N_ERR_STS_PORT_OK) != 0) {
			portok++;
			if (portok >= 50) {
				return 0; /* port must be solid OK */
			}
		} else {
			portok = 0;
			count++;
			if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
				return -EIO;
			}
		}
		udelay(1000);
	}
	return 0;
}

/**
 * keystone_rio_port_init - Configure a RapidIO port
 * @port: index of the port to configure
 * @mode: serdes configuration
 */
static int keystone_rio_port_init(u32 port, u32 mode)
{
	u32 path_mode = _keystone_serdes_config[mode].path_mode[port];

	if (port >= KEYSTONE_RIO_MAX_PORT)
		return -EINVAL;

	/* Send both link request and PNA control symbols (this will clear error states) */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_LONG_CS_TX1(port), 0x2003f044);

	/* Disable packet forwarding */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PF16B_CNTL(port),
		       0xffffffff); 
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PF8B_CNTL(port),
		       0x0003ffff);

	/* Silence and discovery timers */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_SILENCE_TIMER(port),
		       0x20000000);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_DISCOVERY_TIMER(port),
		       0x20000000);

	/* Enable port in input and output */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_CTL(port), 0x600000);

	/* Program channel allocation to ports (1x, 2x or 4x) */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_PATH_CTL(port), path_mode);

	return 0;
}

/**
 * keystone_rio_port_activate - Start using a RapidIO port
 * @port: index of the port to configure
 */
static int keystone_rio_port_activate(u32 port)
{
	u32 val;

	/* Enable interrupt for reset request */
	val = DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_EM_RST_INT_EN);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_EM_RST_INT_EN,
		       val | (1 << port));

	/* Enable all PLM interrupts */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_INT_ENABLE(port), 0xffffffff);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_ALL_INT_EN(port), 1);

	/* Enable all errors */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_RATE_EN(port),
		       0xffffffff);

	/* Cleanup port error status */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_ERR_STAT(port),
		       KEYSTONE_RIO_PORT_ERROR_MASK);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_ERR_DET(port), 0);

	/* Enable promiscuous */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_TLM_SP_CONTROL(port), 0x00309000);

	/* Enable Port-write reception capture */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PW_RX_CAPT(port), 0);

	return 0;
}

static int keystone_rio_init(struct platform_device *pdev)
{
	unsigned int i;
	int          res = 0;

	for (i = 0; i < KEYSTONE_RIO_DBELL_VALUE_MAX; i++)
		init_waitqueue_head(&_keystone_rio.dbell_waitq[i]);
	spin_lock_init(&_keystone_rio.dbell_i_lock);
	mutex_init(&_keystone_rio.lsu_lock);
	init_completion(&_keystone_rio.lsu_completion);

        /* RIO configuration space */
	rio_regs      = (u32) ioremap(KEYSTONE_RIO_REG_BASE, KEYSTONE_RIO_REG_SIZE);
	rio_conf_regs = rio_regs + KEYSTONE_RIO_CONF_SPACE; 

	/* Initialize MP RXU map table resources */
	memset(&keystone_rio_rxu_map_res, 0, sizeof(struct resource));
	keystone_rio_rxu_map_res.start = KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_RXU_MAP_START;
	keystone_rio_rxu_map_res.end   = KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_RXU_MAP_END;
	keystone_rio_rxu_map_res.flags = RIO_RESOURCE_MEM;

	return res;
}

static int keystone_rio_release(void) {
	keystone_rio_mp_exit(0);
	keystone_rio_interrupt_release();
	return 0;
}

/*--------------------------------- Interrupt management ----------------------------*/

static irqreturn_t lsu_interrupt_handler(int irq, void *data)
{
	struct keystone_rio_data *p_rio = (struct keystone_rio_data *) data;
	u32 pending_lsu_int            =
		DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU0_ICSR);

	if (pending_lsu_int & KEYSTONE_RIO_ICSR_LSU0(0))
		complete(&p_rio->lsu_completion);

	/* ACK the interrupt */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU0_ICCR, pending_lsu_int);

	/* Re-arm interrupt */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_INTDST_RATE_CNTL(KEYSTONE_LSU_RIO_INT), 0);

	return IRQ_HANDLED;
}

static void special_interrupt_handler(int ics)
{
	u32 port;
	u32 error;
	
	/* Acknowledge the interrupt */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_ERR_RST_EVNT_ICCR, 1 << ics);

	DPRINTK("ics = %d\n", ics);

	switch(ics) {
	case KEYSTONE_RIO_MCAST_EVT_INT:
		/* Multi-cast event control symbol interrupt received on any port */
		break;

	case KEYSTONE_RIO_PORT_WRITEIN_INT:
		/* Port-write-in request received on any port */
		keystone_rio_port_write_handler(&_keystone_rio);
		break;

	case KEYSTONE_RIO_EVT_CAP_ERROR_INT:
		/* Logical layer error management event capture */
		break;

	case KEYSTONE_RIO_PORT0_ERROR_INT:
	case KEYSTONE_RIO_PORT1_ERROR_INT:
	case KEYSTONE_RIO_PORT2_ERROR_INT:
	case KEYSTONE_RIO_PORT3_ERROR_INT:
		/* Port error */
		port   = (ics - KEYSTONE_RIO_PORT0_ERROR_INT);
		error  = DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_ERR_STAT(port));

		DPRINTK("port = %d, error = 0x%x\n", port, error);

		/* acknowledge error on this port */		
		DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_ERR_STAT(port),
			       error & ~KEYSTONE_RIO_PORT_ERROR_MASK);
		break;
			
	case KEYSTONE_RIO_RESET_INT:
		/* Device reset interrupt on any port */
		break;

	}
	return;
}

static irqreturn_t rio_interrupt_handler(int irq, void *data)
{
	struct keystone_rio_data *p_rio = (struct keystone_rio_data *) data;

	u32 pending_err_rst_evnt_int   =
		DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_ERR_RST_EVNT_ICSR)
		& KEYSTONE_RIO_ERR_RST_EVNT_MASK;

	/* Handle special interrupts (error, reset, special event) */
	while (pending_err_rst_evnt_int) {
		u32 ics = __ffs(pending_err_rst_evnt_int); 
		pending_err_rst_evnt_int &= ~(1 << ics);
		special_interrupt_handler(ics);
	}

	/* Call doorbell handler */
	dbell_handler(p_rio);

	/* Re-arm the interrupt */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_INTDST_RATE_CNTL(KEYSTONE_GEN_RIO_INT),
		       0);
	
	return IRQ_HANDLED;
}

/*
 * Map a sRIO event to a sRIO interrupt
 */
static void keystone_rio_interrupt_map(u32 reg, u32 mask, u32 rio_int)
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
static void keystone_rio_interrupt_setup(void)
{	
	int i;

	/* Clear all pending interrupts */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DOORBELL0_ICCR,     0x0000ffff);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DOORBELL1_ICCR,     0x0000ffff);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DOORBELL2_ICCR,     0x0000ffff);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DOORBELL3_ICCR,     0x0000ffff);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU0_ICCR,          0xffffffff);
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_ERR_RST_EVNT_ICCR,  0x00010f07);

	/* LSU interrupts are routed to RIO interrupt dest 0 (LSU) */
	keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU0_ICRR0, 0x11111111, KEYSTONE_LSU_RIO_INT);
	keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU0_ICRR1, 0x11111111, KEYSTONE_LSU_RIO_INT);
	keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU0_ICRR2, 0x11111111, KEYSTONE_LSU_RIO_INT);
        keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU0_ICRR3, 0x11111111, KEYSTONE_LSU_RIO_INT);
        keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU1_ICRR0, 0x11111111, KEYSTONE_LSU_RIO_INT);

	/* Doorbell interrupts are routed to RIO interrupt dest 1 (Rx/Tx) */
	keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DOORBELL0_ICRR,  0x11111111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DOORBELL0_ICRR2, 0x11111111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DOORBELL1_ICRR,  0x11111111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DOORBELL1_ICRR2, 0x11111111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DOORBELL2_ICRR,  0x11111111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DOORBELL2_ICRR2, 0x11111111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DOORBELL3_ICRR,  0x11111111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DOORBELL3_ICRR2, 0x11111111, KEYSTONE_GEN_RIO_INT);

	/* Error, reset and special event interrupts are routed to RIO interrupt dest 1 (Rx/Tx) */
	keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_ERR_RST_EVNT_ICRR,  0x00000111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_ERR_RST_EVNT_ICRR2, 0x00001111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_ERR_RST_EVNT_ICRR3, 0x00000001, KEYSTONE_GEN_RIO_INT);

	/* The doorbell interrupts routing table is for the 16 general purpose interrupts */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_INTERRUPT_CTL, 0x1);

	/* Do not use pacing */
	for (i = 0; i < 15; i++)
		DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_INTDST_RATE_CNTL(i),
			       0x0);

	/* Attach interrupt handlers */
	request_irq(KEYSTONE_GEN_RIO_EVT,
		    rio_interrupt_handler,
		    0,
		    "sRIO",
		    &_keystone_rio);

	request_irq(KEYSTONE_LSU_RIO_EVT,
		    lsu_interrupt_handler,
		    0,
		    "sRIO LSU",
		    &_keystone_rio);
}

static void keystone_rio_interrupt_release(void)
{
	free_irq(KEYSTONE_LSU_RIO_EVT, &_keystone_rio);
	free_irq(KEYSTONE_GEN_RIO_EVT, &_keystone_rio);
}

/*---------------------------------- Direct I/O -------------------------------*/

static u32 keystone_rio_dio_get_lsu_cc(u32 lsu_id, u8 ltid, u8* lcb)
{
	u32 idx;
	u32 shift;
	u32 value;
	u32 cc;
	/* lSU shadow register status mapping */
	u32 lsu_index[8] = { 0, 9, 15, 20, 24, 33, 39, 44 };

	/* Compute LSU stat index from LSU id and LTID */
	idx   = (lsu_index[lsu_id] + ltid) >> 3;
	shift = ((lsu_index[lsu_id] + ltid) & 0x7) << 2;

	/* Get completion code and context */
	value  = DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_STAT_REG(idx));
	cc     = (value >> (shift + 1)) & 0x7;
	*lcb   = (value >> shift) & 0x1;

	return cc;
}

static u32 keystone_rio_dio_packet_type(int dio_mode)
{
	switch (dio_mode) {
	case RIO_DIO_MODE_READ:
		return KEYSTONE_RIO_PACKET_TYPE_NREAD;
	case RIO_DIO_MODE_WRITER:
		return KEYSTONE_RIO_PACKET_TYPE_NWRITE_R;
	case RIO_DIO_MODE_WRITE:
		return KEYSTONE_RIO_PACKET_TYPE_NWRITE;
	case RIO_DIO_MODE_SWRITE:
		return KEYSTONE_RIO_PACKET_TYPE_SWRITE;
	}
	return KEYSTONE_RIO_PACKET_TYPE_NREAD;
}

/*
 * DIO transfer using LSU directly 
 */
static inline int keystone_rio_dio_raw_transfer(int index,
						u16 dest_id,
						u32 src_addr,
						u32 tgt_addr,
						int size_bytes,
						int size,
						int dio_mode)
{
	unsigned int count;
	unsigned int status      = 0;
	unsigned int res         = 0;
	unsigned int retry_count = KEYSTONE_RIO_RETRY_CNT;
	u8           context;
	u8           ltid;

	size_bytes &= (KEYSTONE_RIO_MAX_DIO_PKT_SIZE - 1);

retry_transfer:

	mutex_lock(&_keystone_rio.lsu_lock);

	INIT_COMPLETION(_keystone_rio.lsu_completion);

	/* Check is there is space in the LSU shadow reg and that it is free */
	count = 0;
	while(1)
        {
		status = DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG6(0));
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0)
		    && ((status & KEYSTONE_RIO_LSU_BUSY_MASK) == 0x0))
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			DPRINTK("cannot get a free LSU\n");
			res = -EIO;
			goto out;
		}
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
        }

	/* Get LCB and LTID, LSU reg 6 is already read */
	context = (status >> 4) & 0x1;
	ltid    = status & 0xf;

	/* LSU Reg 0 - MSB of destination */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG0(0), 0);
	
	/* LSU Reg 1 - LSB of destination */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG1(0), tgt_addr);
	
	/* LSU Reg 2 - source address */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG2(0), src_addr);
	
	/* LSU Reg 3 - Byte count */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG3(0), size_bytes);
	
	/* LSU Reg 4 - 
	 * out port ID = rio.port
         * priority = LSU_PRIO
	 * XAM = 0
	 * ID size = 8 or 16 bit
	 * Dest ID specified as arg
	 * interrupt request = 1 */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG4(0),
		       ((index_to_port(index) << 8)
			| (KEYSTONE_RIO_LSU_PRIO << 4)
			| (size ? (1 << 10) : 0)
			| ((u32) dest_id << 16)
			| 1));

	/* LSU Reg 5 -
	 * doorbell info = 0 for this packet type
	 * hop count = 0 for this packet type
	 * Writing this register should initiate the transfer */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG5(0), 
		       keystone_rio_dio_packet_type(dio_mode));

        /* Wait for transfer to complete */
	res = wait_for_completion_timeout(&_keystone_rio.lsu_completion,
					  msecs_to_jiffies(KEYSTONE_RIO_TIMEOUT_MSEC));
	if (res == 0) {
		/* Timeout expired */
		res = -EIO;
		goto out;
	}

	DPRINTK("LSU interrupt received\n");

	/* Retrieve our completion code */
   	count = 0;
	res   = 0;
	while(1) {
		u8 lcb;
		status = keystone_rio_dio_get_lsu_cc(0, ltid, &lcb);
		if (lcb == context)
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			res = -EIO;
			break;
		}
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
	}
out:		
	mutex_unlock(&_keystone_rio.lsu_lock);

	if (res) {
		DPRINTK("LSU error = %d\n", res);
		return res;
	}

	DPRINTK("status = 0x%x\n", status);

	switch (status & KEYSTONE_RIO_LSU_CC_MASK) {
	case KEYSTONE_RIO_LSU_CC_TIMEOUT:
	case KEYSTONE_RIO_LSU_CC_XOFF:
	case KEYSTONE_RIO_LSU_CC_ERROR:
	case KEYSTONE_RIO_LSU_CC_INVALID:
	case KEYSTONE_RIO_LSU_CC_DMA:
		res = -EIO;
		/* LSU Reg 6 - Flush this transaction */
		DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG6(0), 1);
		break;
	case KEYSTONE_RIO_LSU_CC_RETRY:
	case KEYSTONE_RIO_LSU_CC_CANCELED:
		res = -EAGAIN;
		break;
	default:
		break;
	}

	/* 
	 * Try to transfer again in case of retry doorbell receive
	 * or canceled LSU transfer.
	 */
	if ((res == -EAGAIN) && (retry_count-- > 0)) {
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
		goto retry_transfer;
	}

	return res;
}

/**
 * keystone_rio_dio_transfer - Transfer bytes data from/to DSP address
 * to device ID's global address.
 * @mport: RapidIO master port info
 * @index: ID of the RapidIO interface
 * @dest_id: destination device id 
 * @src_add.r: source (host) address
 * @tgt_addr: target global address
 * @size_bytes: size in bytes
 * @dio_mode: DIO transfer mode (write, write_r, swrite , read)
 *
 * Return %0 on success. Return %-EIO, %-EBUSY or %-EAGAIN on failure.
 */
static int keystone_rio_dio_transfer(struct rio_mport *mport,
				     int index,
				     u16 dest_id,
				     u32 src_addr,
				     u32 tgt_addr,
				     int size_bytes,
				     int dio_mode)
{
	int count  = size_bytes;
	int length;
	int res    = 0;
	u32 s_addr = src_addr;
	u32 t_addr = tgt_addr;

	/* SWRITE implies double-word alignment for address and size */
	if ((dio_mode == RIO_DIO_MODE_SWRITE) &&
	    ((size_bytes % 8) || (tgt_addr % 8) || (src_addr % 8)))
		return -EINVAL;

	/* Sync data if needed */
	if (dio_mode != RIO_DIO_MODE_READ)
		keystone_rio_data_sync_write((u32) src_addr,
					     (u32) src_addr + size_bytes);

	DPRINTK("transferring chunk addr = 0x%x, size = %d\n", src_addr, size_bytes);

	/* Transfer packet by packet */
	while(count) {
		length = (count <= KEYSTONE_RIO_MAX_DIO_PKT_SIZE) ? 
			count : KEYSTONE_RIO_MAX_DIO_PKT_SIZE;
		
		res = keystone_rio_dio_raw_transfer(index,
						    dest_id,
						    s_addr,
						    t_addr,
						    length,
						    mport->sys_size,
						    dio_mode);
		if (res)
			break;
		
		s_addr += length;
		t_addr += length;
		
		count -= length;
	}
	
	/* Sync data if needed */
	if (dio_mode == RIO_DIO_MODE_READ)
		keystone_rio_data_sync_read((u32) src_addr,
					    (u32) src_addr + size_bytes);
	
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

static void dbell_handler(struct keystone_rio_data *p_rio)
{
	u16 pending_dbell;
	u16 ack_dbell;
	unsigned int i;
 
	for (i = 0; i < KEYSTONE_RIO_DBELL_NUMBER; i++) {
		pending_dbell = 
			(u16) DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DOORBELL0_ICSR +
					     ((KEYSTONE_RIO_DOORBELL1_ICSR - KEYSTONE_RIO_DOORBELL0_ICSR) * i));
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
			DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_DOORBELL0_ICCR + 
				       ((KEYSTONE_RIO_DOORBELL1_ICSR - KEYSTONE_RIO_DOORBELL0_ICSR) * i),
				       (u32) ack_dbell);
	}
}

/**
 * keystone_rio_dbell_wait - Wait a KeyStone doorbell message
 * @mport: RapidIO master port info
 * @data: 16-bit info field of RapidIO doorbell message
 *
 * Wait a KeyStone doorbell message.
 */
static int keystone_rio_dbell_wait(struct rio_mport *mport, u16 data)
{
	unsigned long flags;
	u32           dbnum = data & KEYSTONE_RIO_DBELL_MASK;

	DECLARE_WAITQUEUE(wait, current);

	spin_lock_irqsave(&_keystone_rio.dbell_i_lock, flags);

	add_wait_queue(&_keystone_rio.dbell_waitq[dbnum], &wait);

	current->state = TASK_INTERRUPTIBLE;
	spin_unlock_irqrestore(&_keystone_rio.dbell_i_lock, flags);

	schedule();
	
	spin_lock_irqsave(&_keystone_rio.dbell_i_lock, flags);
	remove_wait_queue(&_keystone_rio.dbell_waitq[dbnum], &wait);
	current->state = TASK_RUNNING;
	spin_unlock_irqrestore(&_keystone_rio.dbell_i_lock, flags);

	if (signal_pending(current)) {
		return -ERESTARTSYS;
	}	
	
	DPRINTK("Receiving doorbell (info = %d)\n", dbnum);
	
	return dbnum;
}

/**
 * keystone_rio_doorbell_send - Send a KeyStone doorbell message
 * @mport: RapidIO master port info
 * @index: ID of the RapidIO interface
 * @destid: device ID of target device
 * @num: doorbell number
 *
 * Sends a KeyStone doorbell message. Returns %0 on success or
 * %-EINVAL, %-EIO, %-EBUSY or %-EAGAIN on failure.
 */
static int keystone_rio_dbell_send(struct rio_mport *mport,
				   int index, 
				   u16 dest_id, 
				   u16 num)
{
	unsigned int count;
	unsigned int status = 0;
	unsigned int res    = 0;
	/* Transform doorbell number into info field */
	u16          info   = (num & 0xf) | (((num >> 4) & 0x3) << 5);
	u8           context;
	u8           ltid;

	DPRINTK("Sending doorbell (info = %d) to %x\n", info & KEYSTONE_RIO_DBELL_MASK, dest_id);

	mutex_lock(&_keystone_rio.lsu_lock);

	INIT_COMPLETION(_keystone_rio.lsu_completion);

	/* Check is there is space in the LSU shadow reg and that it is free */
	count = 0;
	while(1)
        {
		status = DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG6(0));
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0)
		    && ((status & KEYSTONE_RIO_LSU_BUSY_MASK) == 0x0))
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			res = -EIO;
			goto out;
		}
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
        }

	/* Get LCB and LTID, LSU reg 6 is already read */
	context = (status >> 4) & 0x1;
	ltid    = status & 0xf;

	/* LSU Reg 0 - MSB of destination */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG0(0), 0);

	/* LSU Reg 1 - LSB of destination */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG1(0), 0);

	/* LSU Reg 2 - source address */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG2(0), 0);

	/* LSU Reg 3 - byte count */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG3(0), 0);

	/* LSU Reg 4 - */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG4(0),
		       ((index_to_port(index) << 8)
			| (KEYSTONE_RIO_LSU_PRIO << 4)
			| ((mport->sys_size) ? (1 << 10) : 0)
			| ((u32) dest_id << 16)
			| 1));

	/* LSU Reg 5 
	 * doorbell info = info
	 * hop count = 0
	 * Packet type = 0xa0 ftype = 10, ttype = 0 */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG5(0),
		       ((info & 0xffff) << 16) | (KEYSTONE_RIO_PACKET_TYPE_DBELL & 0xff));
	
        /* Wait for transfer to complete */
	res = wait_for_completion_timeout(&_keystone_rio.lsu_completion,
					  msecs_to_jiffies(KEYSTONE_RIO_TIMEOUT_MSEC));
	if (res == 0) {
		/* Timeout expired */
		res = -EIO;
		goto out;
	}

	DPRINTK("LSU interrupt received\n");

	/* Retrieve our completion code */
   	count = 0;
	res   = 0;
	while(1) {
		u8 lcb;
		status = keystone_rio_dio_get_lsu_cc(0, ltid, &lcb);
		if (lcb == context)
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			res = -EIO;
			break;
		}
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
	}
out:		
	mutex_unlock(&_keystone_rio.lsu_lock);

	if (res) {
		DPRINTK("LSU error = %d\n", res);
		return res;
	}

	DPRINTK("status = 0x%x\n", status);

	switch (status & KEYSTONE_RIO_LSU_CC_MASK) {
	case KEYSTONE_RIO_LSU_CC_TIMEOUT:
	case KEYSTONE_RIO_LSU_CC_XOFF:
	case KEYSTONE_RIO_LSU_CC_ERROR:
	case KEYSTONE_RIO_LSU_CC_INVALID:
	case KEYSTONE_RIO_LSU_CC_DMA:
		res = -EIO;
		/* LSU Reg 6 - Flush this transaction */
		DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG6(0), 1);
		break;
	case KEYSTONE_RIO_LSU_CC_RETRY:
	case KEYSTONE_RIO_LSU_CC_CANCELED:
		res = -EAGAIN;
		break;
	default:
		DPRINTK("Doorbell sent\n");
		break;
	}

	return res;
}

/*---------------------- Maintenance Request Management  ---------------------*/

/**
 * maint_request - Perform a maintenance request
 * @index: ID of the RapidIO interface
 * @destid: destination ID of target device
 * @hopcount: hopcount for this request
 * @offset: offset in the RapidIO configuration space
 * @len: length of the data 
 * @paddr: physical address of the data on the host
 * @size: 1 for 16bit, 0 for 8bit ID size
 * @type: packet type
 *
 * Returns %0 on success or %-EINVAL, %-EIO, %-EAGAIN or %-EBUSY on failure.
 */
static inline int maint_request(int index,
				u32 dest_id, 
				u8  hopcount,
				u32 offset,
				int len,
				u32 paddr,
				u16 size,
				u16 type)
{
	unsigned int count;
	unsigned int status = 0;
	unsigned int res    = 0;
	u8           context;
	u8           ltid;

	mutex_lock(&_keystone_rio.lsu_lock);

	/* Check is there is space in the LSU shadow reg and that it is free */
	count = 0;
	while(1)
        {
		status = DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG6(0));
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0)
		    && ((status & KEYSTONE_RIO_LSU_BUSY_MASK) == 0x0))
			break;
		count++;

		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			DPRINTK("no LSU available, status = 0x%x\n", status);
			res = -EIO;
			goto out;
		}
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
        }

	/* Get LCB and LTID, LSU reg 6 is already read */
	context = (status >> 4) & 0x1;
	ltid    = status & 0xf;

	/* LSU Reg 0 - MSB of RapidIO address */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG0(0), 0);

	/* LSU Reg 1 - LSB of destination */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG1(0), offset);

	/* LSU Reg 2 - source address */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG2(0), paddr);
	
	/* LSU Reg 3 - byte count */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG3(0), len);
	
	/* LSU Reg 4 - */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG4(0),
		       ((index_to_port(index) << 8)
			| (KEYSTONE_RIO_LSU_PRIO << 4)
			| (size ? (1 << 10) : 0)
			| ((u32) dest_id << 16)));

	/* LSU Reg 5 */
	DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_LSU_REG5(0),
		       ((hopcount & 0xff) << 8) | (type & 0xff));

	/* Retrieve our completion code */
   	count = 0;
	res   = 0;
	while(1) {
		u8 lcb;
		status = keystone_rio_dio_get_lsu_cc(0, ltid, &lcb);
		if (lcb == context)
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			DPRINTK("timeout %d, lcb = %d, cc = %d\n", count, lcb, status);
			res = -EIO;
			break;
		}
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
	}
out:
	mutex_unlock(&_keystone_rio.lsu_lock);

	if (res)
		return res;

	if (status) {
		DPRINTK("transfer error = 0x%x\n", status);
	}

	switch (status) {
	case KEYSTONE_RIO_LSU_CC_TIMEOUT:
	case KEYSTONE_RIO_LSU_CC_XOFF:
	case KEYSTONE_RIO_LSU_CC_ERROR:
	case KEYSTONE_RIO_LSU_CC_INVALID:
	case KEYSTONE_RIO_LSU_CC_DMA:
		return -EIO;
		break;
	case KEYSTONE_RIO_LSU_CC_RETRY:
		return -EBUSY;
		break;
	case KEYSTONE_RIO_LSU_CC_CANCELED:
		return -EAGAIN;
		break;
	default:
		break;
	}
	return 0;
}

/*------------------------- Configuration space mngt  ----------------------*/

/**
 * keystone_local_config_read - Generate a KeyStone local config space read
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be read into
 *
 * Generates a KeyStone local configuration space read. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int keystone_local_config_read(struct rio_mport *mport, 
				      int index, u32 offset, int len, u32 * data)
{
	/* 
	 * Workaround for rionet: the processing element features must content
	 * RIO_PEF_INB_MBOX and RIO_PEF_INB_DOORBELL bits that cannot be set on
	 * KeyStone hardware. So cheat the read value in this case...
	 */
	if (unlikely(offset == RIO_PEF_CAR))
		*data = rio_pe_feat;
	else
		*data = DEVICE_REG32_R(rio_conf_regs + offset);
	
	DPRINTK("index %d offset 0x%x data 0x%x\n", index, offset, *data);

	return 0;
}

/**
 * keystone_local_config_write - Generate a KeyStone local config space write
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be written
 *
 * Generates a KeyStone local configuration space write. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int keystone_local_config_write(struct rio_mport *mport, 
				       int index, u32 offset, int len, u32 data)
{
	DPRINTK("index %d offset 0x%x data 0x%x\n", index, offset, data);
	DEVICE_REG32_W(rio_conf_regs + offset, data);

	return 0;
}

/**
 * keystone_rio_config_read - Generate a KeyStone read maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Location to be read into
 *
 * Generates a KeyStone read maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int
keystone_rio_config_read(struct rio_mport *mport, int index, u16 destid,
			 u8 hopcount, u32 offset, int len, u32* val)
{
	u32* tbuf;
	int res;

	tbuf = (u32*) kzalloc(L1_CACHE_ALIGN(len), GFP_KERNEL);
	if (!tbuf)
		return -ENOMEM;

	res = maint_request(index, destid, hopcount, offset, len,
			    virt_to_phys(tbuf), mport->sys_size,
			    KEYSTONE_RIO_PACKET_TYPE_MAINT_R);

	keystone_rio_data_sync_read((u32) tbuf,
				    (u32) tbuf + L1_CACHE_ALIGN(len));

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

	DPRINTK("index %d destid %d hopcount %d offset 0x%x len %d val 0x%x res %d\n",
		index, destid, hopcount, offset, len, *val, res);

	return res;
}

/**
 * keystone__rio_config_write - Generate a KeyStone write maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Value to be written
 *
 * Generates an KeyStone write maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int
keystone_rio_config_write(struct rio_mport *mport, int index, u16 destid, u8 hopcount,
			  u32 offset, int len, u32 val)
{
	u32* tbuf;
	int res;

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

	keystone_rio_data_sync_write((u32) tbuf,
				     (u32) tbuf + L1_CACHE_ALIGN(len));

	res = maint_request(index, destid, hopcount, offset, len,
			    virt_to_phys(tbuf), mport->sys_size,
			    KEYSTONE_RIO_PACKET_TYPE_MAINT_W);

	DPRINTK("index %d destid %d hopcount %d offset 0x%x len %d val 0x%x res %d\n",
		index, destid, hopcount, offset, len, val, res);

	kfree(tbuf);

	return res;
}

/*------------------------------- Port-Write management --------------------------*/

/**
 * keystone_rio_pw_enable - enable/disable port-write interface init
 * @mport: Master port implementing the port write unit
 * @enable: 1=enable; 0=disable port-write message handling
 */
static int keystone_rio_pwenable(struct rio_mport *mport, int enable)
{
	/* Enable/Disable port-write-in interrupt */
	return 0;
}

static void keystone_rio_pw_dpc(struct work_struct *work)
{
	struct keystone_rio_data *p_rio = container_of(work, struct keystone_rio_data, pw_work);
	unsigned long flags;
	u32 msg_buffer[RIO_PW_MSG_SIZE/sizeof(u32)];

	/*
	 * Process port-write messages
	 */
	spin_lock_irqsave(&p_rio->pw_fifo_lock, flags);
	while (kfifo_out(&p_rio->pw_fifo, 
			 (unsigned char *) msg_buffer,
			 RIO_PW_MSG_SIZE)) {

		/* Process one message */
		spin_unlock_irqrestore(&p_rio->pw_fifo_lock, flags);

#ifdef KEYSTONE_RIO_DEBUG_PW

		{
			u32 i;
			printk("%s : Port-Write Message:", __func__);
			for (i = 0; i < RIO_PW_MSG_SIZE/sizeof(u32); i++) {
				if ((i % 4) == 0)
					printk("\n0x%02x: 0x%08x", i*4,
					       msg_buffer[i]);
				else
					printk(" 0x%08x", msg_buffer[i]);
			}
			printk("\n");
		}
#endif /* KEYSTONE_RIO_DEBUG_PW */

		/* Pass the port-write message to RIO core for processing */
		rio_inb_pwrite_handler((union rio_pw_msg *) msg_buffer);
		spin_lock_irqsave(&p_rio->pw_fifo_lock, flags);
	}
	spin_unlock_irqrestore(&p_rio->pw_fifo_lock, flags);
}

/**
 *  keystone_rio_port_write_handler - KeyStone port write interrupt handler
 *
 * Handles port write interrupts. Parses a list of registered
 * port write event handlers and executes a matching event handler.
 */
static void keystone_rio_port_write_handler(struct keystone_rio_data *p_rio)
{
	int pw;

	/* Check that we have a port-write-in case */
	pw = DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PW_RX_STAT) & 0x1;

	/* Schedule deferred processing if PW was received */
	if (pw) {
		/* 
		 * Retrieve PW message
		 */
		p_rio->port_write_msg.msg.em.comptag =
			DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PW_RX_CAPT(0));
		p_rio->port_write_msg.msg.em.errdetect =
			DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PW_RX_CAPT(1));
		p_rio->port_write_msg.msg.em.is_port =
			DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PW_RX_CAPT(2));
		p_rio->port_write_msg.msg.em.ltlerrdet =
			DEVICE_REG32_R(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PW_RX_CAPT(3));

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
			printk(KERN_WARNING "RIO: ISR Discarded Port-Write Msg(s) (%d)\n",
			       p_rio->port_write_msg.discard_count);
		}
		schedule_work(&p_rio->pw_work);
	}

	/* Acknowledge port-write-in */
	return;
}

/**
 * keystone_rio_port_write_init - KeyStone port write interface init
 * @mport: Master port implementing the port write unit
 *
 * Initializes port write unit hardware and buffer
 * ring. Called from keystone_rio_setup(). Returns %0 on success
 * or %-ENOMEM on failure.
 */
static int keystone_rio_port_write_init(struct keystone_rio_data *p_rio)
{
	int i;

	/* Following configurations require a disabled port write controller */
	keystone_rio_pwenable(NULL, 0);

	/* Clear port-write-in capture registers */
	for (i = 0; i < 4; i++)
		DEVICE_REG32_W(KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_PW_RX_CAPT(i),
			       0x00000000);

	INIT_WORK(&p_rio->pw_work, keystone_rio_pw_dpc);
	spin_lock_init(&p_rio->pw_fifo_lock);
	if (kfifo_alloc(&p_rio->pw_fifo, RIO_PW_MSG_SIZE * 32, GFP_KERNEL)) {
		printk(KERN_WARNING "RIO: FIFO allocation failed\n");
		return -ENOMEM;
	}
	return 0;
}

/*----------------------------- Message passing management  ----------------------*/

static irqreturn_t txu_interrupt_handler(int irq, void *data);

/*
 * This macro defines the mapping from Linux RIO mailbox to stream id for type 9 packet
 * Let use an one-to-one mapping for the time being, may be adjusted here if needed.
 */
#define mbox_to_strmid(mbox) ((mbox) & 0xffff)

/*
 * Cleanup queues used by the QMSS/PKTDMA
 */
static void keystone_rio_cleanup_qs(int queue)
{
	struct qm_host_desc *hd = NULL;
	while ((hd = hw_qm_queue_pop(queue)) != NULL)
		hw_qm_queue_push(hd, _keystone_rio.free_queue, DEVICE_QM_DESC_SIZE_BYTES);
}

/*
 * Global shutdown of RapidIO MP
 */
static void keystone_rio_mp_exit(int idx)
{
	int mbox;

	DPRINTK("shutting down RXU PktDMA\n");

	(void) pktdma_rx_disable(_keystone_rio.rx_cfg);

	for (mbox = 0; mbox < _keystone_rio.max_mbox; mbox++) {
		keystone_rio_cleanup_qs(_keystone_rio.rx_free_queues[mbox]);
		keystone_rio_cleanup_qs(_keystone_rio.rx_queues[mbox]);
	}

	DPRINTK("shutting down TXU PktDMA\n");

	(void) pktdma_tx_disable(_keystone_rio.tx_cfg);

	keystone_rio_cleanup_qs(_keystone_rio.tx_cp_queue);

	free_irq(_keystone_rio.tx_irq, &_keystone_rio);
}

/*
 * Global initialization of RapidIO MP
 */
static int keystone_rio_mp_init(u32 max_mbox,
				u32 rxu_queues,
				u32 txu_queue,
				u32 rxu_irqs,
				u32 txu_irq) 
{
	struct keystone_rio_data *rio = &_keystone_rio;
	u32 mbox;
	int res;

	if (max_mbox == 0)
		return 0;

	if ((max_mbox < 0) || (max_mbox > KEYSTONE_RIO_MAX_MBOX))
		return -EINVAL;

	/* Initialize PktDMA */
	pktdma_region_init(KEYSTONE_RIO_CDMA_BASE,
			   KEYSTONE_RIO_CDMA_SIZE,
			   &keystone_rio_cdma_base_addr);
	
	if (keystone_rio_cdma_base_addr == NULL)
		return -ENOMEM;
	
	rio->rx_cfg = (struct pktdma_rx_cfg*) kmalloc(sizeof(struct pktdma_rx_cfg), GFP_KERNEL);
	if (rio->rx_cfg == NULL) {
		return -ENOMEM;
		goto out_error;
	}
	
	rio->tx_cfg = (struct pktdma_tx_cfg*) kmalloc(sizeof(struct pktdma_tx_cfg), GFP_KERNEL);
	if (rio->tx_cfg == NULL) {
		res = -ENOMEM;
		goto out_error;
	}

	/* 
	 * Will use dts in the near future, let use platform defines instead for the
	 * time being.
	 */
	rio->max_mbox        = max_mbox;
	rio->rx_channel      = KEYSTONE_RIO_CDMA_RX_FIRST_CHANNEL;
	rio->tx_channel      = KEYSTONE_RIO_CDMA_TX_FIRST_CHANNEL;
	rio->rx_flow         = KEYSTONE_RIO_CDMA_RX_FIRST_FLOW;
	rio->free_queue      = DEVICE_QM_RIO_FREE_Q;
	rio->tx_queue        = DEVICE_QM_RIO_TX_Q;
	rio->tx_cp_queue     = txu_queue;
	rio->rx_irqs         = rxu_irqs;
	rio->tx_irq          = txu_irq;

	for (mbox = 0; mbox < max_mbox; mbox++) {
		rio->rx_queues[mbox]      = rxu_queues + mbox;
		rio->rx_free_queues[mbox] = DEVICE_QM_RIO_RX_FREE_Q + mbox;
	}

	/* Configure Rx PKTDMA */
	rio->rx_cfg->base_addr           = (u32) keystone_rio_cdma_base_addr;
	rio->rx_cfg->rx_base_offset      = KEYSTONE_RIO_CDMA_RX_CHAN_CFG_OFFSET;
	rio->rx_cfg->rx_chan             = rio->rx_channel;
	rio->rx_cfg->n_rx_chans          = max_mbox; /* We need one channel per mbox */
	rio->rx_cfg->flow_base_offset    = KEYSTONE_RIO_CDMA_RX_FLOW_CFG_OFFSET;
	rio->rx_cfg->rx_flow             = rio->rx_flow;
	rio->rx_cfg->n_rx_flows          = max_mbox; /* We need one flow per mbox */
	rio->rx_cfg->qmnum_free_buf      = 0;
	rio->rx_cfg->queue_free_buf      = &rio->rx_free_queues[0];
	rio->rx_cfg->qmnum_rx            = 0;
	rio->rx_cfg->queue_rx            = &rio->rx_queues[0];
	rio->rx_cfg->tdown_poll_count    = KEYSTONE_RIO_CDMA_RX_TIMEOUT_COUNT;

	if (pktdma_rx_config(rio->rx_cfg)) {
		DPRINTK("Setup of PktDMA Rx failed\n");
		res = -EBUSY;
		goto out_error;
	}

	/* Configure Tx PKTDMA */
	rio->tx_cfg->base_addr           = (u32) keystone_rio_cdma_base_addr;
	rio->tx_cfg->gbl_ctl_base_offset = KEYSTONE_RIO_CDMA_GLOBAL_CFG_OFFSET;
	rio->tx_cfg->tx_base_offset      = KEYSTONE_RIO_CDMA_TX_CHAN_CFG_OFFSET;
	rio->tx_cfg->tx_chan             = rio->tx_channel;
	rio->tx_cfg->n_tx_chans	         = 1;

	if (pktdma_tx_config(rio->tx_cfg)) {
		DPRINTK("Setup of PktDMA Tx failed\n");
		res = -EBUSY;
		goto out_error;
	}

	/* Initialize the QMSS queues */
	for (mbox = 0; mbox < max_mbox; mbox++) {
		keystone_rio_cleanup_qs(rio->rx_queues[mbox]);
		keystone_rio_cleanup_qs(rio->rx_free_queues[mbox]);
	}

	keystone_rio_cleanup_qs(rio->tx_cp_queue);

	request_irq(rio->tx_irq,
		    txu_interrupt_handler,
		    0,
		    "sRIO TXU",
		    (void *) rio);
	return 0;

out_error:
	if (rio->rx_cfg)
		kfree(rio->rx_cfg);

	if (rio->tx_cfg)
		kfree(rio->tx_cfg);

	return res;
}

/*
 * TXU interrupt handler
 */
static irqreturn_t txu_interrupt_handler(int irq, void *data)
{
	struct keystone_rio_data *rio = (struct keystone_rio_data*) data;
	struct qm_host_desc *hd;

	DPRINTK("Acknowledge transmitted message (irq = %d)\n", irq);

	while((hd = hw_qm_queue_pop(rio->tx_cp_queue)) != NULL) {
		struct keystone_rio_desc_info *rio_desc =
			(struct keystone_rio_desc_info *) &hd->ps_data;
		int mbox_id = rio_desc->mbox;
		struct keystone_rio_mbox_info *mbox = &keystone_rio_tx_mbox[mbox_id];
		struct rio_mport *port = mbox->port;
		void *dev_id = mbox->dev_id;
		
		DPRINTK("release desc = 0x%x, mbox = %d, slot = %d\n",
			(u32) hd, mbox_id, mbox->slot);

		if (mbox->running) {
			/*
			 * Client is in charge of freeing the associated buffers
			 * Because we do not have explicit hardware ring but queues, we
			 * do not know where we are in the sw ring, let use fake slot.
			 * But the semantic hereafter is dangerous in case of re-order:
			 * bad buffer may be released...
			 */
			port->outb_msg[mbox_id].mcback(port, dev_id, mbox_id, mbox->slot++);
			if (mbox->slot > mbox->entries)
				mbox->slot = 0;
		}
		
		/* Give back descriptor to the free queue */
		hw_qm_queue_push(hd, rio->free_queue, DEVICE_QM_DESC_SIZE_BYTES);
	}
	return IRQ_HANDLED;
}

/*
 * RXU interrupt handler
 */
static irqreturn_t rxu_interrupt_handler(int irq, void *data)
{
	struct keystone_rio_mbox_info *mbox = (struct keystone_rio_mbox_info *) data;
	DPRINTK("Received message for mbox = %d\n", mbox->id);

	if (mbox->running) {
		/* Client callback (slot is not used) */
		mbox->port->inb_msg[mbox->id].mcback(mbox->port, mbox->dev_id,
						     mbox->id, 0);
	}
	return IRQ_HANDLED;
}

/**
 * keystone_rio_map_mbox - Map a mailbox to a given queue.
 * for both type 11 and type 9 packets.
 * @mbox: mailbox to map
 * @queue: associated queue number
 *
 * Returns %0 on success or %-ENOMEM on failure.
 */
static int keystone_rio_map_mbox(int mbox, int queue, int flowid, int size)
{
	struct keystone_rio_mbox_info *rx_mbox = &keystone_rio_rx_mbox[mbox];
	u32 mapping_entry_low;
	u32 mapping_entry_high;
	u32 mapping_entry_qid;
	u32 mapping_t9_reg0;
	u32 mapping_t9_reg1;
	u32 mapping_t9_reg2;
	u32 t9_offset = KEYSTONE_RIO_RXU_T9_MAP_START - KEYSTONE_RIO_RXU_MAP_START;
	int res;

	/* Map the multi-segment mailbox to the corresponding Rx queue for type 11 */
	mapping_entry_low = ((mbox & 0x1f) << 16)
 		| (0x3f000000); /* Given mailbox, all letters, srcid = 0 */

	mapping_entry_high = KEYSTONE_RIO_MAP_FLAG_SEGMENT /* multi-segment messaging */
		| KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC
		| KEYSTONE_RIO_MAP_FLAG_DST_PROMISC;       /* promiscuous (don't care about src/dst id) */

	/* Map the multi-segment mailbox for type 9 as well */
	mapping_t9_reg0 = 0;                               /* accept all COS and srcid = 0 */
	mapping_t9_reg1 = KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC
		| KEYSTONE_RIO_MAP_FLAG_DST_PROMISC;       /* promiscuous (don't care about src/dst id) */
	mapping_t9_reg2 = (0xffff << 16) | (mbox_to_strmid(mbox)); /* given stream id */

	/* Set TT flag */
	if (size) {
		mapping_entry_high |= KEYSTONE_RIO_MAP_FLAG_TT_16;
		mapping_t9_reg1    |= KEYSTONE_RIO_MAP_FLAG_TT_16;
	}

	/* QMSS/PktDMA mapping */
	mapping_entry_qid = (queue & 0x3fff) | (flowid << 16);

	/* Allocate two look-up table entries for mbox to queue mapping */
	res = allocate_resource(&keystone_rio_rxu_map_res,
				&rx_mbox->map_res,
				24,
				KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_RXU_MAP_START,
				KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_RXU_MAP_END,
				0xc,
				NULL,
				NULL);
	if (res)
		return -ENOMEM;

	DPRINTK("Using RXU map entries 0x%x to 0x%x, mbox = %d, flowid = %d, queue = %d\n", 
		rx_mbox->map_res.start,
		rx_mbox->map_res.end,
		mbox, flowid, queue);

	DEVICE_REG32_W(rx_mbox->map_res.start,     mapping_entry_low);
	DEVICE_REG32_W(rx_mbox->map_res.start + 4, mapping_entry_high);
	DEVICE_REG32_W(rx_mbox->map_res.start + 8, mapping_entry_qid);

	DEVICE_REG32_W(rx_mbox->map_res.start + t9_offset,     mapping_t9_reg0);
	DEVICE_REG32_W(rx_mbox->map_res.start + t9_offset + 4, mapping_t9_reg1);
	DEVICE_REG32_W(rx_mbox->map_res.start + t9_offset + 8, mapping_t9_reg2);

        /*
	 *  The RapidIO peripheral looks at the incoming RapidIO msgs
	 *  and if there is only one segment (the whole msg fits into one
	 *  RapidIO msg), the peripheral uses the single segment mapping
	 *  table. Therefore we need to map the single-segment mailbox too.
	 *  The same Rx CPPI Queue is used (as for the multi-segment
	 *  mailbox).
	 */
	mapping_entry_high &= ~KEYSTONE_RIO_MAP_FLAG_SEGMENT;
	
	DEVICE_REG32_W(rx_mbox->map_res.start + 12, mapping_entry_low);
	DEVICE_REG32_W(rx_mbox->map_res.start + 16, mapping_entry_high);
	DEVICE_REG32_W(rx_mbox->map_res.start + 20, mapping_entry_qid);

	return 0;
}

/**
 * rio_open_inb_mbox - Initialize KeyStone inbound mailbox
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
	struct keystone_rio_mbox_info *rx_mbox = &keystone_rio_rx_mbox[mbox];
	int res;

	DPRINTK("mport = 0x%x, dev_id = 0x%x, mbox = %d, entries = %d\n",
		(u32) mport, (u32) dev_id, mbox, entries);
	
	/* Check if the port is already registered in this queue */
	if (rx_mbox->port == mport)
		return 0;
	
	rx_mbox->dev_id  = dev_id;
	rx_mbox->entries = entries;
	rx_mbox->port    = mport;
	rx_mbox->id      = mbox;
	rx_mbox->running = 1;

	/* Map the mailbox to queue/flow */
	res = keystone_rio_map_mbox(mbox,
				    _keystone_rio.rx_queues[mbox],
				    _keystone_rio.rx_flow,
				    mport->sys_size);
	if (res)
		return res;

	request_irq(_keystone_rio.rx_irqs + mbox,
		    rxu_interrupt_handler,
		    0,
		    "sRIO RXU",
		    (void*) rx_mbox);
	return 0;
}

/**
 * rio_close_inb_mbox - Shut down KeyStone inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, stop queues and free all resources
 */
void rio_close_inb_mbox(struct rio_mport *mport, int mbox) 
{
	struct keystone_rio_mbox_info *rx_mbox = &keystone_rio_rx_mbox[mbox];

	DPRINTK("mport = 0x%x, mbox = %d\n", (u32) mport, mbox);

       	rx_mbox->running = 0;
	
	if (rx_mbox->port) {
		int res;
		rx_mbox->port = NULL;
		
		keystone_rio_cleanup_qs(_keystone_rio.rx_free_queues[mbox]);
		keystone_rio_cleanup_qs(_keystone_rio.rx_queues[mbox]);

		/* Release associated resource */
		res = release_resource(&rx_mbox->map_res);
		if (res)
			printk("release of resource 0x%x failed", rx_mbox->map_res.start);

		free_irq(_keystone_rio.rx_irqs + mbox, (void *)rx_mbox);
	}
	return;
}

/**
 * rio_open_outb_mbox - Initialize KeyStone outbound mailbox
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
	struct keystone_rio_mbox_info *tx_mbox = &keystone_rio_tx_mbox[mbox];

	DPRINTK("mport = 0x%x, dev_id = 0x%x, mbox = %d, entries = %d\n",
		(u32) mport, (u32) dev_id, mbox, entries);

	/* Check if already initialized */
	if (tx_mbox->port == mport)
		return 0;

	tx_mbox->dev_id  = dev_id;
	tx_mbox->entries = entries;
	tx_mbox->port    = mport;
	tx_mbox->id      = mbox;
	tx_mbox->slot    = 0;
	tx_mbox->running = 1;

	return 0;
}

/**
 * rio_close_outb_mbox - Shut down KeyStone outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, stop queues and free all resources
 */
void rio_close_outb_mbox(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_mbox_info *tx_mbox = &keystone_rio_tx_mbox[mbox];
	DPRINTK("mport = 0x%x, mbox = %d\n", (u32) mport, mbox);
	tx_mbox->port    = NULL;
	tx_mbox->running = 0;

	return;
}

/**
 * rio_hw_add_outb_message - Add a message to the KeyStone outbound message queue
 * @mport: Master port with outbound message queue
 * @rdev: Target of outbound message
 * @mbox: Outbound mailbox
 * @buffer: Message to add to outbound queue
 * @len: Length of message
 *
 * Adds the @buffer message to the KeyStone outbound message queue. Returns
 * %0 on success or %-EBUSY on failure.
 */
int rio_hw_add_outb_message(struct rio_mport *mport, struct rio_dev *rdev, 
			    int mbox, void *buffer, const size_t len)
{
	struct qm_host_desc *hd;
	struct keystone_rio_desc_info *rio_desc;
	int  loop  = 0;
	u32  paddr = virt_to_phys((u32) buffer);
	u32  plen  = ((len + 7) & ~0x7);

	if (len == 0)
		return -EINVAL;

	/* Spin a little bit if free queue is empty until we get free desc */
	while(((hd = hw_qm_queue_pop(_keystone_rio.free_queue)) == NULL)
	      && (++loop < KEYSTONE_RIO_LOOP_FREE_QUEUE));

	if (hd == NULL) {
		DPRINTK("no free descriptor retrieved\n");
		return -EBUSY;
	}

	/* Sync data if needed */
	keystone_rio_data_sync_write((u32) buffer,
				     (u32) buffer + len);

	/* Fill descriptor */
	QM_DESC_DINFO_SET_PKT_LEN(hd->desc_info, plen);
	QM_DESC_DINFO_SET_PSINFO_LOC(hd->desc_info, 0);
	QM_DESC_TINFO_SET_S_TAG_LO(hd->tag_info, 0);

	rio_desc = (struct keystone_rio_desc_info *) &hd->ps_data;

	/* Word 1: source id and dest id (common to packet 11 and packet 9) */
	rio_desc->ps_desc[0] = (rdev->destid & 0xffff) | (mport->host_deviceid << 16);

	if (_keystone_rio.mbox_type == RIO_PACKET_TYPE_MESSAGE) {
		/* Packet 11 case (Message) */
		QM_DESC_DINFO_SET_PKT_TYPE(hd->desc_info, 31); /* 31 for type11, 30 for type9 */
		
		/* Word 2: ssize = 32 dword, 4 retries, letter = 0, mbox */
		rio_desc->ps_desc[1] = (KEYSTONE_RIO_MSG_SSIZE << 17) | (4 << 21)
			| (mbox & 0x1f);
	} else {
		/* Packet 9 case (Data Streaming) */
		QM_DESC_DINFO_SET_PKT_TYPE(hd->desc_info, 30); /* 30 for type9 */
       
		/* Word 2: COS = 0, stream id based on mbox */
		rio_desc->ps_desc[1] = (mbox_to_strmid(mbox) << 16);
	}

	if (rdev->net->hport->sys_size)
		rio_desc->ps_desc[1] |= KEYSTONE_RIO_DESC_FLAG_TT_16; /* tt */

	DPRINTK("packet type %d, ps_desc[0] = 0x%x, ps_desc[1] = 0x%x\n",
		_keystone_rio.mbox_type, rio_desc->ps_desc[0], rio_desc->ps_desc[1]);

	/* Word 3 is private data, use it to store the mbox */
	rio_desc->mbox = mbox;

	hd->buff_len	  = plen;
	hd->orig_buff_len = plen;
	hd->buff_ptr	  = paddr;
	hd->next_bdptr    = 0;
	hd->orig_buff_ptr = paddr;

	DPRINTK("transmitting packet of len %d (paddr=0x%x, hd=0x%x) to queue = %d (return queue = %d)\n",
		plen, paddr, (u32) hd, _keystone_rio.tx_queue, _keystone_rio.tx_cp_queue);

	/* Return the descriptor back to the tx completion queue */
	QM_DESC_PINFO_SET_QM(hd->packet_info, 0);
	QM_DESC_PINFO_SET_SIZE(hd->packet_info, 2); /* sRIO protocol needs 2 words */
	QM_DESC_PINFO_SET_QUEUE(hd->packet_info, _keystone_rio.tx_cp_queue);
	
	hw_qm_queue_push(hd, _keystone_rio.tx_queue, DEVICE_QM_DESC_SIZE_BYTES);

	return 0;
}

/**
 * rio_hw_add_inb_buffer - Add buffer to the KeyStone inbound message queue
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 * @buf: Buffer to add to inbound queue
 *
 * Adds the @buf buffer to the KeyStone inbound message queue. Returns
 * %0 on success or %-EINVAL on failure.
 */
int rio_hw_add_inb_buffer(struct rio_mport *mport, int mbox, void *buffer)
{
	struct qm_host_desc *hd;
	int  loop = 0;
	u32  paddr = virt_to_phys((u32) buffer);

	/* Spin a little bit if free queue is empty until we get free desc */
	while(((hd = hw_qm_queue_pop(_keystone_rio.free_queue)) == NULL)
	      && (++loop < KEYSTONE_RIO_LOOP_FREE_QUEUE));

	if (hd == NULL) {
		DPRINTK("no free descriptor retrieved\n");
		return -EBUSY;
	}

	/* Fill descriptor */
	QM_DESC_DINFO_SET_PSINFO_LOC(hd->desc_info, 0);
	QM_DESC_TINFO_SET_S_TAG_LO(hd->tag_info, 0);

	hd->buff_len	  = KEYSTONE_RIO_MSG_MAX_BUFFER_SIZE;
	hd->orig_buff_len = KEYSTONE_RIO_MSG_MAX_BUFFER_SIZE;
	hd->buff_ptr	  = paddr;
	hd->next_bdptr    = 0;
	hd->orig_buff_ptr = paddr;

	DPRINTK("adding in receive queue packet (paddr=0x%x, hd=0x%x) to queue = %d\n",
		paddr, (u32) hd, _keystone_rio.rx_free_queues[mbox]);

	hw_qm_queue_push(hd, _keystone_rio.rx_free_queues[mbox],
			 DEVICE_QM_DESC_SIZE_BYTES);

	return 0;
}

/**
 * rio_hw_get_inb_message - Fetch inbound message from the KeyStone message unit
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 *
 * Gets the next available inbound message from the inbound message queue.
 * A pointer to the message is returned on success or NULL on failure.
 */
void *rio_hw_get_inb_message(struct rio_mport *mport, int mbox)
{
	struct qm_host_desc *hd;
	struct keystone_rio_desc_info *rio_desc;
	u32 paddr;
	u32 len;
	void *buff;
	
	hd = hw_qm_queue_pop(_keystone_rio.rx_queues[mbox]);
	
	if (hd == NULL)
		return NULL;

	rio_desc = (struct keystone_rio_desc_info *) &hd->ps_data;

	DPRINTK("Fetching message 0x%x, mbox = %d\n",
		(u32) rio_desc, mbox);

	/* If mailbox is not our, re-queue the message but it should not happen */
	if (unlikely(rio_desc->mbox != mbox)) {
		DPRINTK("Discarding message, mbox = %d, expected = %d \n",
			rio_desc->mbox, mbox);
		hw_qm_queue_push(hd, _keystone_rio.rx_queues[rio_desc->mbox],
				 DEVICE_QM_DESC_SIZE_BYTES);
	}
	
	paddr = hd->buff_ptr;
	buff  = phys_to_virt(paddr);
	len   = QM_DESC_DINFO_GET_PKT_LEN(hd->desc_info);

	/* Sync data if needed */
	if (buff && len)
		keystone_rio_data_sync_read((u32) buff, (u32) buff + len);

	/* Give back descriptor to the free queue */
	hw_qm_queue_push(hd, _keystone_rio.free_queue, DEVICE_QM_DESC_SIZE_BYTES);

	return buff;
}

EXPORT_SYMBOL_GPL(rio_hw_add_outb_message);
EXPORT_SYMBOL_GPL(rio_hw_add_inb_buffer);
EXPORT_SYMBOL_GPL(rio_hw_get_inb_message);

/*-------------------------- Main Linux driver functions -----------------------*/

static char *cmdline_hdid      = NULL;
static char *cmdline_ports     = NULL;
static char *cmdline_init      = NULL;
static char *cmdline_mode      = NULL;
static char *cmdline_size      = NULL;
static char *cmdline_mbox_type = NULL;

static int keystone_rio_get_hdid(int index, int default_id, int size)
{
	int id;

        if (!cmdline_hdid) 
                id = default_id;
	else {
		id = simple_strtol(cmdline_hdid, NULL, 0);
	}

	if (id < 0)
		/* Read the default one */
		id = keystone_rio_read_hdid(size);
	return id;
}

static int keystone_rio_get_ports(int index, int default_ports)
{
	int ports = 0;

        if (!cmdline_ports) 
                ports = default_ports;
	else {
		int port[5];
		int i;

		get_options(cmdline_ports, ARRAY_SIZE(port), port);

		if (port[0] > 4) {
			printk("incorrect number of parameters for rioports\n");
			return default_ports;
		}
		
		for (i = 0; i < port[0]; i++) {
			if (port[i+1] == -1)
				break;
			ports |= (1 << port[i+1]);
		}
	}
	return ports;
}

static int keystone_rio_get_init(int index, int default_init)
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

static int keystone_rio_get_mode(int index, int default_mode)
{
	int mode = 0;

        if ((!cmdline_mode) || (!get_option(&cmdline_mode, &mode)))
                mode = default_mode;

	return mode;
}

static int keystone_rio_get_size(int index, int default_size)
{
	int size = 0;

        if ((!cmdline_size) || (!get_option(&cmdline_size, &size)))
                size = default_size;

	return size;
}

static int keystone_rio_mbox_type(int index, int default_type)
{
	int type = 0;

        if (!cmdline_mbox_type) 
                type = default_type;
	else {
		if (strcmp(cmdline_mbox_type, "message") == 0)
			type = RIO_PACKET_TYPE_MESSAGE;
		else if (strcmp(cmdline_mbox_type, "streaming") == 0)
			type = RIO_PACKET_TYPE_STREAM;
		else 
			type = default_type;
	}
	return type;
}

static int keystone_rio_get_cmdline_hdid(char *s)
{
        if (!s)
                return 0;
        cmdline_hdid = s;
        return 1;
}

__setup("riohdid=", keystone_rio_get_cmdline_hdid);

static int keystone_rio_get_cmdline_ports(char *s)
{
        if (!s)
                return 0;
        cmdline_ports = s;
        return 1;
}

__setup("rioports=", keystone_rio_get_cmdline_ports);

static int keystone_rio_get_cmdline_init(char *s)
{
        if (!s)
                return 0;
        cmdline_init = s;
        return 1;
}
__setup("rioinit=", keystone_rio_get_cmdline_init);

static int keystone_rio_get_cmdline_mode(char *s)
{
        if (!s)
                return 0;
        cmdline_mode = s;
        return 1;
}
__setup("riomode=", keystone_rio_get_cmdline_mode);

static int keystone_rio_get_cmdline_size(char *s)
{
        if (!s)
                return 0;
        cmdline_size = s;
        return 1;
}
__setup("riosize=", keystone_rio_get_cmdline_size);

static int keystone_rio_get_cmdline_mbox_type(char *s)
{
        if (!s)
                return 0;
        cmdline_mbox_type = s;
        return 1;
}
__setup("riombox-type=", keystone_rio_get_cmdline_mbox_type);

/**
 * keystone_rio_fixup - Architecture specific fixup fonction
 * @mport: RapidIO master port info
 * @rdev: Associated RIO dev structure
 */
static int keystone_rio_fixup(struct rio_mport *mport, struct rio_dev *rdev)
{
	rdev->dio.base_offset = RAM_MEMORY_START; /* For EVM test purpose */

	/* Add flow control for this device */
	keystone_add_flow_control(rdev);

	return 0;
}

int keystone_rio_register_mport(u32 port_id, u32 hostid, u32 init, u32 size)
{
	struct rio_ops   *ops;
	struct rio_mport *port;

	ops = kzalloc(sizeof(struct rio_ops), GFP_KERNEL);

	ops->lcread       = keystone_local_config_read;
	ops->lcwrite      = keystone_local_config_write;
	ops->cread        = keystone_rio_config_read;
	ops->cwrite       = keystone_rio_config_write;
	ops->dsend        = keystone_rio_dbell_send;
	ops->dwait        = keystone_rio_dbell_wait;
	ops->transfer     = keystone_rio_dio_transfer;
	ops->fixup        = keystone_rio_fixup;
	ops->pwenable     = keystone_rio_pwenable;

	port = kzalloc(sizeof(struct rio_mport), GFP_KERNEL);
	port->id          = port_id;
	port->index       = port_to_index(port_id);
	INIT_LIST_HEAD(&port->dbells);

	/* Make a dummy per port region as ports are not really separated on KeyStone */ 
	port->iores.start = KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_LM_REQ(port_id);
	port->iores.end   = KEYSTONE_RIO_REG_BASE + KEYSTONE_RIO_SP_CTL(port_id);
	port->iores.flags = IORESOURCE_MEM;

	rio_init_dbell_res(&port->riores[RIO_DOORBELL_RESOURCE], 0, 0xffff);
	rio_init_mbox_res(&port->riores[RIO_INB_MBOX_RESOURCE], 0, 0);
	rio_init_mbox_res(&port->riores[RIO_OUTB_MBOX_RESOURCE], 0, 0);

	sprintf(port->name, "RIO%d mport", port_id);

	port->ops           = ops;
	port->init          = init;
	port->host_deviceid = hostid;
	port->sys_size      = size;
	port->phy_type      = RIO_PHY_SERIAL;

	rio_register_mport(port);

	return 0;
}

/*
 * Platform configuration setup
 */
static int keystone_rio_setup_controller(struct platform_device *pdev)
{
     	struct keystone_rio_board_controller_info *c;
	struct device *dev;
	u32 hostid;
	u32 ports;
	u32 p;
	u32 init;
	u32 mode;
	u32 size = 0;
	int res = 0;
	int idx = pdev->id - 1;

	dev = get_device(&pdev->dev);
	c   = (struct keystone_rio_board_controller_info *) dev->platform_data;

	size   = keystone_rio_get_size(idx, c->size);
	hostid = keystone_rio_get_hdid(idx, c->id, size);
	ports  = keystone_rio_get_ports(idx, c->ports);
	init   = keystone_rio_get_init(idx, c->init);
	mode   = keystone_rio_get_mode(idx, c->mode);

	DPRINTK("size = %d, hostid = %d, ports = 0x%x, init = %d, mode = %d\n",
		size, hostid, ports, init, mode);

	_keystone_rio.mbox_type = keystone_rio_mbox_type(idx, c->mbox_type);

	_keystone_serdes_config = c->serdes_config;

	if (mode >= c->serdes_config_num) {
		mode = 0;
		printk(KERN_WARNING "RIO: invalid port mode, forcing it to %d\n", mode);
	}

	printk(KERN_WARNING "RIO: register sRIO controller for hostid %d\n", hostid);

	/* Hardware set up of the controller */
	keystone_rio_hw_init(mode, hostid);

	/* Initialize port write interface */
	res = keystone_rio_port_write_init(&_keystone_rio);
	if (res)
		return res;

	/* 
	 * Configure all ports even if we do not use all of them.
	 * This is needed for 2x and 4x modes.
	 */
	for (p = 0; p < KEYSTONE_RIO_MAX_PORT; p++) {
		res = keystone_rio_port_init(p, mode);
		if (res < 0) {
			printk(KERN_WARNING "RIO: initialization of port %d failed\n", p);
			return res;
		}
	}

	/* Global initialization of RapidIO MP */
	res = keystone_rio_mp_init(c->max_mbox,
				   c->rxu_queues,
				   c->txu_queue,
				   c->rxu_irqs,
				   c->txu_irq);
	if (res)
		return res;	

	/* Initialize interrupts */
	keystone_rio_interrupt_setup();

	/* Start the controller */
	keystone_rio_start();

	/* Use and check ports status (but only the requested ones) */
	p = ports;
	while(p) {
		int status;
		u32 port = __ffs(p);
		p &= ~(1 << port);

		/* Start the port */
		keystone_rio_port_activate(port);

		/* 
		 * Check the port status here before calling the generic RapidIO
		 * layer. Port status check is done in rio_mport_is_active() as 
		 * well but we need to do it our way first due to some delays in
		 * hw initialization.
		 */
		status = keystone_rio_port_status(port);
		if (status == 0) {
			/* Register this port  */
			res = keystone_rio_register_mport(port, hostid, init, size);
			if (res)
				goto out;
			
			printk(KERN_WARNING "RIO: port %d registered\n", port);
		} else
			printk(KERN_WARNING "RIO: port %d not ready\n", port);
	}
out:
	return res;
}

static int __init keystone_rio_probe(struct platform_device *pdev)	
{
	int res;

	/* sRIO main driver (global ressources, interrupts) */
	res = keystone_rio_init(pdev);
	if (res < 0)
		return res;

	printk(banner);

	/* Setup the sRIO controller */
	res = keystone_rio_setup_controller(pdev);
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

	return 0;
}

static int __exit keystone_rio_remove(struct platform_device *pdev)
{
	keystone_rio_release();

#ifdef CONFIG_RAPIDIO_DEV
	rio_dev_exit();
#endif
	put_device(&pdev->dev);
	return 0;
}

static struct platform_driver keystone_rio_driver  = {
	.driver = {
		.name	= "keystone-rapidio",
		.owner	= THIS_MODULE,
	},
	.probe	= keystone_rio_probe,
	.remove = __exit_p(keystone_rio_remove),
};

static int keystone_rio_module_init(void)
{
	return platform_driver_register(&keystone_rio_driver);
}

static void keystone_rio_module_exit(void)
{
	platform_driver_unregister(&keystone_rio_driver);
}

subsys_initcall(keystone_rio_module_init);
module_exit(keystone_rio_module_exit);

MODULE_AUTHOR("Aurelien Jacquiot");
MODULE_DESCRIPTION("TI KeyStone RapidIO device driver");
MODULE_LICENSE("GPL");
