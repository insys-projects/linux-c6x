/*
 * Copyright (C) 2011 Texas Instruments Incorporated
 * Author: Sandeep Paulraj <s-paulraj@ti.com>
 *         Aurelien Jacquiot <a-jacquiot@ti.com>
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

#ifndef KEYSTONE_PKTDMA_H
#define KEYSTONE_PKTDMA_H

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
	u32 rsvd0[6];
};

struct pktdma_rx_channel_config {
	u32 rx_channel_global_config_reg_a;
	u32 rsvd0[7];
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

	/* Base address for the PKTDMA overlay registers */

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

/*
 * Configuration parameter structures
 */
struct pktdma_rx_cfg {
	u32 base_addr;          /* Base address of registers */
	u32 rx_base_offset;	/* Offset of rx registers */
	u32 rx_chan;            /* Index of first rx channel */
	u32 n_rx_chans;		/* The number of rx channels */
	u32 flow_base_offset;	/* Offset of flow registers */
	u32 rx_flow;            /* Index of first flow */
	u32 n_rx_flows;		/* Number of rx flows */
	u32 qmnum_free_buf;	/* Queue manager for descriptors/buffers for received packets */
	u32 *queue_free_buf;	/* Queue that holds descriptors/buffers for received packets */
	u32 qmnum_rx;		/* Queue manager for received packets */
	u32 *queue_rx;		/* Default Rx queue for received packets */
	u32 tdown_poll_count;	/* Number of loop iterations to wait for teardown */
	u8  use_acc;            /* Use accumulators if non zero */
	u8  acc_threshold;      /* Accumulator threshold */
	u32 acc_channel;        /* Accumulator channel */
	u32 acc_list_addr;      /* Virtual address of the accumulator list memory */
        u32 acc_list_phys_addr; /* Physical address of the accumulator list memory */
}; 

struct pktdma_tx_cfg {
	u32 base_addr;          /* Base address of registers */
	u32 gbl_ctl_base_offset; /* Offset of global control registers */
	u32 tx_base_offset;	/* Offset of the tx registers */
	u32 tx_chan;            /* Index of first tx channel */
	u32 n_tx_chans;		/* The number of tx channels */
	u32 queue_tx;		/* Default Tx queue for Tx completion packets */
	u8  use_acc;            /* Use accumulators if non zero */
	u8  acc_threshold;      /* Accumulator threshold */
	u32 acc_channel;        /* Accumulator channel */
	u32 acc_list_addr;      /* Virtual address of the accumulator list memory */
        u32 acc_list_phys_addr; /* Physical address of the accumulator list memory */
};

/* 
 * Prototypes
 */
int pktdma_rx_disable(struct pktdma_rx_cfg *cfg);
int pktdma_tx_disable(struct pktdma_tx_cfg *cfg);
int pktdma_rx_config(struct pktdma_rx_cfg *cfg);
int pktdma_tx_config(struct pktdma_tx_cfg *cfg);
void pktdma_flow_config(struct pktdma_rx_cfg *cfg,
			int flow,
			u32 queue_rx,
			u32 queue_free_buf);
int pktdma_region_init(u32 base_paddr,
		       u32 size,
		       void __iomem **vaddr);

#endif /* KEYSTONE_PKTDMA_H */
