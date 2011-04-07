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

/* 
 * PDMA Channel type
 */
enum pktdma_chtype
{
	/* Receive Channel */
	PKTDMA_CHTYPE_RX_CHANNEL = 0,
	/* Transmit Channel */
	PKTDMA_CHTYPE_TX_CHANNEL
};

/* 
 * PDMA Channel Enable
 */
enum pktdma_chstate
{
	/* Disable Channel */
	PKTDMA_CHSTATE_CHANNEL_DISABLE = 0,
	/* Enable Channel */
	PKTDMA_CHSTATE_CHANNEL_ENABLE
};

/*
 * PDMA Wait after Channel Teardown
 */
enum pktdma_wait
{
	/* No wait */
	PKTDMA_WAIT_NO_WAIT = 0,
	/* Wait */
	PKTDMA_WAIT_WAIT
};

enum pktdma_mod
{
	/* SRIO */
	PKTDMA_SRIO = 0,
	/* AIF */
	PKTDMA_AIF,
	/* FFTC A */
	PKTDMA_FFTC_A,
	/* FFTC B */
	PKTDMA_FFTC_B,
	/* PASS */
	PKTDMA_PA_SS,
	/* QMSS */
	PKTDMA_QMSS
};

struct pktdma_global_control {
	u32 revision;
	u32 perf_control;
	u32 emulation_control;
	u32 priority_control;
	u32 qm_base_address[4];
};

struct pktdma_tx_channel_config {
	u32 tx_channel_global_config_reg_a;
	u32 tx_channel_global_config_reg_b;
	u32  rsvd0[6];
};

struct pktdma_rx_channel_config {
	u32 rx_channel_global_config_reg_a;
	u32  rsvd0[7];
};

struct pktdma_rx_flow_config {
	u32 rx_flow_config_reg_a;
	u32 rx_flow_config_reg_b;
	u32 rx_flow_config_reg_c;
	u32 rx_flow_config_reg_d;
	u32 rx_flow_config_reg_e;
	u32 rx_flow_config_reg_f;
	u32 rx_flow_config_reg_g;
	u32 rx_flow_config_reg_h;
};

struct pktdma_tx_scheduler_config {
	u32 tx_channel_scheduler_config_reg;
};

/*
 * PDMA global configuration structure
 */
struct pktdma_global_config
{
	/* PKTDMA this configuration belongs to */
	enum pktdma_mod	dmanum;
	/* Maximum supported Rx Channels */
	u32		maxrxch;
	/* Maximum supported Tx Channels */
	u32		maxtxch;
	/* Maximum supported Rx Flows */
	u32		maxrxflow;
	/* Priority for all Rx transactions of this PKTDMA */
	u8		rxpriority;
	/* Priority for all Tx transactions of this PKTPDMA */
	u8		txpriority;

	/* Base address for the CPDMA overlay registers */

	/* Global Config registers */
	struct pktdma_global_control		*gblcfg;
	/* Tx Channel Config registers */
	struct pktdma_tx_channel_config		*txch;
	/* Rx Channel Config registers */
	struct pktdma_rx_channel_config		*rxch;
	/* Tx Channel Scheduler registers */
	struct pktdma_tx_scheduler_config	*txsched;
	/* Rx Flow Config registers */
	struct pktdma_rx_flow_config		*rxflow;
};

/*
 * Packet DMA configuration structure
 */
struct pktdma_init_cfg
{
	enum pktdma_mod	dmanum;
	u8		writefifodepth;
        u16		timeoutcount;
    
	/* Queue Manager 0 base address register */
	volatile u32            qm0_base_address;
	/* Queue Manager 1 base address register */
	volatile u32            qm1_base_address;
	/* Queue Manager 2 base address register */
	volatile u32            qm2_base_address;
	/* Queue Manager 3 base address register */
	volatile u32            qm3_base_address;
};

/*
 * Packet DMA transmit channel configuration structure
 */
struct pktdma_tx_ch_init_cfg
{
	int			channel_num;
        enum pktdma_chstate	txenable;
	u8			priority;
	u16			filter_epib;
	u16			filter_ps;
	u16			aif_mono_Mode;
};

/* 
 * Packet DMA receive channel configuration structure
 */
struct pktdma_rx_ch_init_cfg
{
	int			channel_num;
        enum pktdma_chstate	rxEnable;
};

/*
 * Packet DMA receive flow configuration structure
 */
struct pktdma_rx_flow_cfg
{
	s16	flow_id_num;
	u16	rx_dest_qnum;
	u16	rx_dest_qmgr;
	u16	rx_sop_offset;
	u16	rx_ps_location;
	u8	rx_desc_type;
	u16	rx_error_handling;
	u16	rx_psinfo_present;
	u16	rx_einfo_present;
	u8	rx_dest_tag_lo;
	u8	rx_dest_tag_hi;
	u8      rx_src_tag_lo;
	u8      rx_src_tag_hi;
        u8	rx_size_thresh0_en;
	u8	rx_size_thresh1_en;
	u8	rx_size_thresh2_en;
	u8	rx_dest_tag_lo_sel;
	u8	rx_dest_tag_hi_sel;
	u8	rx_src_tag_lo_sel;
	u8	rx_src_tag_hi_sel;    
	u16	rx_fdq1_qnum;
	u16	rx_fdq1_qmgr;
	u16	rx_fdq0_sz0_qnum;
	u16	rx_fdq0_sz0_qmgr;
	u16	rx_fdq3_qnum;
	u16	rx_fdq3_qmgr;
	u16	rx_fdq2_qnum;
	u16	rx_fdq2_qmgr;
	u16	rx_size_thresh1;
	u16	rx_size_thresh0;
	u16	rx_fdq0_sz1_qnum;
	u16	rx_fdq0_sz1_qmgr;
	u16	rx_size_thresh2;
	u16	rx_fdq0_sz3_qnum;
	u16	rx_fdq0_sz3_qmgr;
	u16	rx_fdq0_sz2_qnum;
	u16	rx_fdq0_sz2_qmgr;
};

/* Emulation control register */
#define CPDMA_REG_EMU_CTL		0x08

/* CPPI Tx DMA channel control registers */
#define CPDMA_REG_TCHAN_CFG_REG_A(x)	(0x00 + (x) * 0x20)
#define CPDMA_REG_TCHAN_CFG_REG_B(x)	(0x04 + (x) * 0x20)

/* CPPI Rx DMA channel control register */
#define CPDMA_REG_RCHAN_CFG_REG_A(x)    (0x00 + (x) * 0x20)

/* CPPI Tx DMA Scheduler Configuration register */
#define CPDMA_REG_TCHAN_SCHED_CFG(x)	((x) * 0x04)

/* CPPI Rx DMA flow configuration registers */
#define CPDMA_RX_FLOW_CFG(reg,idx)	(((reg) * 4) + ((idx) * 0x20))
#define CPDMA_RX_FLOW_REG_A		0
#define CPDMA_RX_FLOW_REG_B		1
#define CPDMA_RX_FLOW_REG_C		2
#define CPDMA_RX_FLOW_REG_D		3
#define CPDMA_RX_FLOW_REG_E		4
#define CPDMA_RX_FLOW_REG_F		5
#define CPDMA_RX_FLOW_REG_G		6
#define CPDMA_RX_FLOW_REG_H		7

/* Descriptor type created by flows */
#define CPDMA_DESC_TYPE_HOST		1

/* CPPI Tx DMA channel control register A definitions */
#define CPDMA_REG_VAL_TCHAN_A_TX_ENABLE ((u32)1 << 31)
#define CPDMA_REG_VAL_TCHAN_A_TX_TDOWN  (1 << 30)

/* CPPI Tx DMA channel control register B definitions */
#define CPDMA_REG_VAL_TCHAN_B_TX_FILT_EINFO	(1 << 30)
#define CPDMA_REG_VAL_TCHAN_B_TX_FILT_PSWORDS	(1 << 29)
#define CPDMA_REG_TCHAN_B_SET_DEFAULT_TDOWN_QMGR(x,v)  (x) = (BOOT_SET_BITFIELD((x), (v), 13, 12)
#define CPDMA_REG_TCHAN_B_SET_DEFAULT_TDOWN_QNUM(x,v)  (x) = (BOOT_SET_BITFIELD((x), (v), 11,  0)

/* CPPI Rx DMA channel control register A definitions */
#define CPDMA_REG_VAL_RCHAN_A_RX_ENABLE		((u32)1 << 31)
#define CPDMA_REG_VAL_RCHAN_A_RX_TDOWN		(1 << 30)

/*
 * CPPI Tx DMA Scheduler Confuration value. This sets the priorities of
 * the channels. If set to all equal, the actual value doesn't matter
 */
#define CPDMA_REG_VAL_TCHAN_SCHED_HIGH_PRIORITY		0 
#define CPDMA_REG_VAL_TCHAN_SCHED_MED_HIGH_PRIORITY	1 
#define CPDMA_REG_VAL_TCHAN_SCHED_MED_LOW_PRIORITY	2 
#define CPDMA_REG_VAL_TCHAN_SCHED_LOW_PRIORITY		3 

/* CPPI Rx flow configuration register A */
#define CPDMA_REG_VAL_MAKE_RX_FLOW_A(einfo,psinfo,rxerr,desc,psloc,sopOff,qmgr,qnum)  \
	(   ((einfo & 1) << 30)       |   \
            ((psinfo & 1) << 29)      |   \
            ((rxerr & 1) << 28)       |   \
            ((desc & 3) << 26)        |   \
            ((psloc & 1) << 25)       |   \
            ((sopOff & 0x1ff) << 16)  |   \
            ((qmgr & 3) << 12)        |   \
            ((qnum & 0xfff) << 0))
            
/* CPPI Rx flow configuration register B. No tags are used */
#define CPDMA_REG_VAL_RX_FLOW_B_DEFAULT		0

/* CPPI Rx flow configuration register C. No tag replacement and no size thresholds */
#define CPDMA_REG_VAL_RX_FLOW_C_DEFAULT		0

/* CPPI Rx flow configuration register D */
#define CPDMA_REG_VAL_MAKE_RX_FLOW_D(fd0Qm, fd0Qnum, fd1Qm, fd1Qnum)   \
        (   ((fd0Qm & 3) << 28)         |   \
            ((fd0Qnum & 0xfff) << 16)   |   \
            ((fd1Qm & 3) << 12)         |   \
            ((fd1Qnum & 0xfff) <<  0))

/* CPPI Rx flow configuration register E */
#define CPDMA_REG_VAL_RX_FLOW_E_DEFAULT		0

/* CPPI Rx flow configuration register F */
#define CPDMA_REG_VAL_RX_FLOW_F_DEFAULT		0

/* CPPI Rx flow configuration register G */
#define CPDMA_REG_VAL_RX_FLOW_G_DEFAULT		0

/* CPPI Rx flow configuration register H */
#define CPDMA_REG_VAL_RX_FLOW_H_DEFAULT		0
            
/* Default Emulation control register value disables loopback */            
#define CPDMA_REG_VAL_EMU_CTL_NO_LOOPBACK	0\

struct cpdma_rx_cfg {
	u32 rx_base;		/* Base address of rx registers */
	u32 n_rx_chans;		/* The number of rx channels */
	u32 flow_base;		/* Add address of flow registers */
	u32 nrx_flows;		/* Number of rx flows */
	u32 qmnum_free_buf;	/* Queue manager for descriptors/buffers for received packets */
	u32 queue_free_buf;	/* Queue that holds descriptors/buffers for received packets */
	u32 qmnum_rx;		/* Queue manager for received packets */
	u32 queue_rx;		/* Default Rx queue for received packets */
	u32 tdown_poll_count;	/* Number of loop iterations to wait for teardown */
}; 

struct cpdma_tx_cfg {
	u32 gbl_ctl_base;	/* Base address of global control registers */
	u32 tx_base;		/* Base address of the tx registers */
	u32 n_tx_chans;		/* The number of tx channels */
};

/* Prototypes */
int cpdma_rx_disable(struct cpdma_rx_cfg *cfg);
int cpdma_rx_config(struct cpdma_rx_cfg *cfg);
int cpdma_tx_config(struct cpdma_tx_cfg *cfg);
int cpdma_tx_disable(struct cpdma_tx_cfg *cfg);

/* Register offsets */
#define CPGMACSL_REG_ID		0x00
#define CPGMACSL_REG_CTL	0x04
#define CPGMACSL_REG_STATUS	0x08
#define CPGMACSL_REG_RESET	0x0c
#define CPGMACSL_REG_MAXLEN	0x10
#define CPGMACSL_REG_BOFF	0x14
#define CPGMACSL_REG_RX_PAUSE	0x18
#define CPGMACSL_REG_TX_PAURSE	0x1c
#define CPGMACSL_REG_EM_CTL	0x20
#define CPGMACSL_REG_PRI	0x24

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

int mac_sl_reset(u16 port);
int mac_sl_config(u16 port, struct mac_sliver *cfg);

int keystone_pa_enable(struct pa_config *cfg);
int keystone_pa_disable(void);
int keystone_pa_config(u8 *mac_addr);
