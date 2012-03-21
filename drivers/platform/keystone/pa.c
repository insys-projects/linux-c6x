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
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <mach/keystone_qmss.h>
#include <mach/keystone_pa.h>
#include <mach/keystone_netcp.h>
#include <linux/keystone/pa.h>
#include <linux/keystone/qmss.h>
#include <linux/keystone/pktdma.h>

struct pa_priv {
	struct device	*dev;
	u32              tx_queue;
	u32              rx_queue;
	u32		 tx_retqueue; /* pktdma return queue */
	u32		 rx_retqueue; /* pktdma return queue */
	u32              tx_channel;
	u32              rx_channel;
	const char	*tx_chan_name;
	const char	*rx_chan_name;
	u32		 cmd_flow_num;
	u32		 cmd_queue_num;
	u32		 data_flow_num;
	u32		 data_queue_num;
	u32              free_queue;
};

struct pa_packet {
	enum dma_data_direction		 direction;
	struct pa_priv			*priv;
	u32			         chan;
	u32                              queue;
	u32                              retqueue;
	struct qm_host_desc             *desc;
	dma_cookie_t			 cookie;
	u32				 swdata[3];
	u32				 psdata[1];
	void				*data;
	u32                              cmd_size;
};

#define PA_PDSP_CONST_REG_INDEX_C25_C24     0
#define PA_PDSP_CONST_REG_INDEX_C27_C26     1
#define PA_PDSP_CONST_REG_INDEX_C29_C28     2
#define PA_PDSP_CONST_REG_INDEX_C31_C30     3

const u32 pap_pdsp_const_reg_map[6][4] =
{
	/* PDSP0: C24-C31 */
	{
		0x0000007F,		/* C25-C24 */
		0x0000006E,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP1: C24-C31 */
	{
		0x0001007F,		/* C25-C24 */
		0x00480040,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP2: C24-C31 */
	{
		0x0002007F,		/* C25-C24 */
		0x00490044,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP3: C24-C31 */
	{
		0x0003007F,		/* C25-C24 */
		0x0000006E,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP4: C24-C31 */
	{
		0x0070007F,		/* C25-C24 */
		0x00000000,		/* C27-C26 */
		0x04080404,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	},
	/* PDSP5: C24-C31 */
	{
		0x0071007F,		/* C25-C24 */
		0x00000000,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000000		/* C31-C30 */
	}
};

static void __iomem *pa_base;

static inline void pa_write_reg(u32 val, int reg)
{
	__raw_writel(val, pa_base + reg);
}

static inline u32 pa_read_reg(int reg)
{
	return __raw_readl(pa_base + reg);
}

static inline void swizFwd (struct pa_frm_forward *fwd)
{
	fwd->flow_id = fwd->flow_id;
	fwd->queue   = cpu_to_be16(fwd->queue);

	if (fwd->forward_type == PAFRM_FORWARD_TYPE_HOST) {
		fwd->u.host.context      = cpu_to_be32(fwd->u.host.context);
		fwd->u.host.multi_route  = fwd->u.host.multi_route;
		fwd->u.host.multi_idx    = fwd->u.host.multi_idx;
		fwd->u.host.pa_pdsp_router = fwd->u.host.pa_pdsp_router;
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_SA) {
		fwd->u.sa.sw_info_0 = cpu_to_be32(fwd->u.sa.sw_info_0);
		fwd->u.sa.sw_info_1 = cpu_to_be32(fwd->u.sa.sw_info_1);
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_SRIO) {
		fwd->u.srio.ps_info0 = cpu_to_be32(fwd->u.srio.ps_info0);
		fwd->u.srio.ps_info1 = cpu_to_be32(fwd->u.srio.ps_info1);
		fwd->u.srio.pkt_type = fwd->u.srio.pkt_type;
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_ETH) {
		fwd->u.eth.ps_flags	= fwd->u.eth.ps_flags;
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_PA) {
		fwd->u.pa.pa_dest	= fwd->u.pa.pa_dest;
		fwd->u.pa.custom_type	= fwd->u.pa.custom_type;
		fwd->u.pa.custom_idx	= fwd->u.pa.custom_idx;
	}

	fwd->forward_type = fwd->forward_type;
}

static inline void swizFcmd (struct pa_frm_command *fcmd)
{
	fcmd->command_result =  cpu_to_be32(fcmd->command_result);
	fcmd->command	     =  fcmd->command;
	fcmd->magic          =  fcmd->magic;
	fcmd->com_id         =  cpu_to_be16(fcmd->com_id);
	fcmd->ret_context    =  cpu_to_be32(fcmd->ret_context);
	fcmd->reply_queue    =  cpu_to_be16(fcmd->reply_queue);
	fcmd->reply_dest     =  fcmd->reply_dest;
	fcmd->flow_id        =  fcmd->flow_id;
}

static inline void swizAl1 (struct pa_frm_cmd_add_lut1 *al1)
{
	al1->index         =  al1->index;
	al1->type          =  al1->type;
	al1->cust_index    =  al1->cust_index;

	if (al1->type == PAFRM_COM_ADD_LUT1_STANDARD) {
		al1->u.eth_ip.etype = cpu_to_be16(al1->u.eth_ip.etype);
		al1->u.eth_ip.vlan  = cpu_to_be16(al1->u.eth_ip.vlan);
		al1->u.eth_ip.spi   = cpu_to_be32(al1->u.eth_ip.spi);
		al1->u.eth_ip.flow  = cpu_to_be32(al1->u.eth_ip.flow);
	
		if (al1->u.eth_ip.key & PAFRM_LUT1_KEY_MPLS)
			al1->u.eth_ip.pm.mpls     =  cpu_to_be32(al1->u.eth_ip.pm.mpls);
		else {
			al1->u.eth_ip.pm.ports[0] =  cpu_to_be16(al1->u.eth_ip.pm.ports[0]);
			al1->u.eth_ip.pm.ports[1] =  cpu_to_be16(al1->u.eth_ip.pm.ports[1]);
		}

		al1->u.eth_ip.proto_next  =  al1->u.eth_ip.proto_next;
		al1->u.eth_ip.tos_tclass  =  al1->u.eth_ip.tos_tclass;
		al1->u.eth_ip.inport      =  al1->u.eth_ip.inport;
		al1->u.eth_ip.key         =  al1->u.eth_ip.key;
		al1->u.eth_ip.match_flags =  cpu_to_be16(al1->u.eth_ip.match_flags);
	} else if (al1->type == PAFRM_COM_ADD_LUT1_SRIO) {
		al1->u.srio.src_id	= cpu_to_be16(al1->u.srio.src_id);
		al1->u.srio.dest_id	= cpu_to_be16(al1->u.srio.dest_id);
		al1->u.srio.etype	= cpu_to_be16(al1->u.srio.etype);
		al1->u.srio.vlan	= cpu_to_be16(al1->u.srio.vlan);
		al1->u.srio.pri         = al1->u.srio.pri;
		al1->u.srio.type_param1 = cpu_to_be16(al1->u.srio.type_param1);
		al1->u.srio.type_param2 = al1->u.srio.type_param2;
		al1->u.srio.key         = al1->u.srio.key;
		al1->u.srio.match_flags = cpu_to_be16(al1->u.srio.match_flags);
		al1->u.srio.next_hdr    = al1->u.srio.next_hdr;
		al1->u.srio.next_hdr_offset = cpu_to_be16(al1->u.srio.next_hdr_offset);
    
	} else {
		al1->u.custom.etype		=  cpu_to_be16(al1->u.custom.etype);
		al1->u.custom.vlan		=  cpu_to_be16(al1->u.custom.vlan);
		al1->u.custom.key		=  al1->u.custom.key;
		al1->u.custom.match_flags	=  cpu_to_be16(al1->u.custom.match_flags);
	}

	swizFwd(&(al1->match));
	swizFwd(&(al1->next_fail));
}

int pa_conv_routing_info(struct	pa_frm_forward *fwd_info,
				struct	pa_route_info *route_info,
				int cmd_dest, u16 fail_route)
{
	u8 *pcmd = NULL;
	fwd_info->flow_id = route_info->flow_id;
	fwd_info->queue   = route_info->queue;
  
	if (route_info->dest == PA_DEST_HOST) {
		fwd_info->forward_type   = PAFRM_FORWARD_TYPE_HOST;
		fwd_info->u.host.context = route_info->sw_info_0;

		if (route_info->m_route_index >= 0) {
			if (route_info->m_route_index >= PA_MAX_MULTI_ROUTE_SETS) {
			return (PA_ERR_CONFIG);
			}

		fwd_info->u.host.multi_route	= 1;
		fwd_info->u.host.multi_idx	= route_info->m_route_index;
		fwd_info->u.host.pa_pdsp_router	= PAFRM_DEST_PA_M_0;
		}
    
		pcmd = fwd_info->u.host.cmd;
	} else if (route_info->dest == PA_DEST_DISCARD)	{
		fwd_info->forward_type = PAFRM_FORWARD_TYPE_DISCARD;
	} else if (route_info->dest == PA_DEST_EMAC) {
		fwd_info->forward_type = PAFRM_FORWARD_TYPE_ETH;
		fwd_info->u.eth.ps_flags = (route_info->pkt_type_emac_ctrl &
						PA_EMAC_CTRL_CRC_DISABLE)?
						PAFRM_ETH_PS_FLAGS_DISABLE_CRC:0;
		fwd_info->u.eth.ps_flags |= ((route_info->pkt_type_emac_ctrl &
						PA_EMAC_CTRL_PORT_MASK) <<
						PAFRM_ETH_PS_FLAGS_PORT_SHIFT);
  
	} else if (fail_route) {
		return (PA_ERR_CONFIG);

	} else if (((route_info->dest == PA_DEST_CONTINUE_PARSE_LUT1) &&
			(route_info->custom_type != PA_CUSTOM_TYPE_LUT2)) ||
			((route_info->dest == PA_DEST_CONTINUE_PARSE_LUT2) &&
			(route_info->custom_type != PA_CUSTOM_TYPE_LUT1))) {

		/* Custom Error check */
		if (((route_info->custom_type == PA_CUSTOM_TYPE_LUT1) &&
			(route_info->custom_index >= PA_MAX_CUSTOM_TYPES_LUT1)) ||
			((route_info->custom_type == PA_CUSTOM_TYPE_LUT2) &&
			(route_info->custom_index >= PA_MAX_CUSTOM_TYPES_LUT2)))
			return(PA_ERR_CONFIG); 

		fwd_info->forward_type = PAFRM_FORWARD_TYPE_PA;
		fwd_info->u.pa.custom_type = (u8)route_info->custom_type;
		fwd_info->u.pa.custom_idx  = route_info->custom_index; 

		if (route_info->dest == PA_DEST_CONTINUE_PARSE_LUT2) {
			fwd_info->u.pa.pa_dest = PAFRM_DEST_PA_C2;
		} else {
			/*
			 * cmd_dest is provided by calling function 
			 * There is no need to check error case 
			 */
			fwd_info->u.pa.pa_dest = (cmd_dest == PA_CMD_TX_DEST_0)?
				PAFRM_DEST_PA_C1_1:PAFRM_DEST_PA_C1_2;
		}
	} else if (route_info->dest == PA_DEST_SASS) {
		fwd_info->forward_type   = PAFRM_FORWARD_TYPE_SA;
		fwd_info->u.sa.sw_info_0 = route_info->sw_info_0;
		fwd_info->u.sa.sw_info_1 = route_info->sw_info_1;
		pcmd = fwd_info->u.sa.cmd;
	} else if (route_info->dest == PA_DEST_SRIO) {
		fwd_info->forward_type		= PAFRM_FORWARD_TYPE_SRIO;
		fwd_info->u.srio.ps_info0	= route_info->sw_info_0;
		fwd_info->u.srio.ps_info1	= route_info->sw_info_1;
		fwd_info->u.srio.pkt_type	= route_info->pkt_type_emac_ctrl;
	} else {
		return (PA_ERR_CONFIG);
	}
  

	if (pcmd && route_info->pcmd) {
		struct pa_cmd_info *pacmd = route_info->pcmd;
		struct pa_patch_info *patch_info;
		struct pa_cmd_set *cmd_set;
    
		switch (pacmd->cmd) {
			case PA_CMD_PATCH_DATA:
				patch_info = &pacmd->params.patch;
				if ((patch_info->n_patch_bytes > 2) ||
					(patch_info->overwrite) ||
					(patch_info->patch_data == NULL))
					return (PA_ERR_CONFIG); 
          
				pcmd[0] = PAFRM_RX_CMD_CMDSET;
				pcmd[1] = patch_info->n_patch_bytes;
				pcmd[2] = patch_info->patch_data[0];
				pcmd[3] = patch_info->patch_data[1];
				break;
      
			case PA_CMD_CMDSET:
				cmd_set = &pacmd->params.cmd_set;
				if(cmd_set->index >= PA_MAX_CMD_SETS)
					return (PA_ERR_CONFIG); 
				
				pcmd[0] = PAFRM_RX_CMD_CMDSET;
				pcmd[1] = (u8)cmd_set->index; 
				break;
			default:
				return(PA_ERR_CONFIG);
		}    
	}
	return (PA_OK);
}

int keystone_pa_reset(void)
{
	u32 i;

	/* Reset and disable all PDSPs */
	for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
		pa_write_reg(PA_REG_VAL_PDSP_CTL_RESET_PDSP,
			PA_REG_PDSP_CTL(i));

		while((pa_read_reg(PA_REG_PDSP_CTL(i))
		       & PA_REG_VAL_PDSP_CTL_STATE));
	}

	/* Reset packet Id */
	pa_write_reg(1, PA_REG_PKTID_SOFT_RESET);

	/* Reset LUT2 */
	pa_write_reg(1, PA_REG_LUT2_SOFT_RESET);

	/* Reset statistic */
	pa_write_reg(1, PA_REG_STATS_SOFT_RESET);

	/* Reset timers */
	for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
		pa_write_reg(0, PA_REG_TIMER_CTL(i));
	}

	return 0;
}

int keystone_pa_enable(int pdsp)
{
	u32 i;
	int v;

	if ((pdsp < 0) || (pdsp >= DEVICE_PA_NUM_PDSPS))
		return -EINVAL;

	/* Check the PDSP state */
	v = pa_read_reg(PA_REG_PDSP_CTL(pdsp));
	if (v & PA_REG_VAL_PDSP_CTL_STATE) {
		/* Already enabled */
		return 1;
	}

	/* Clear the mailboxes */
	for (i = 0; i < PA_NUM_MAILBOX_SLOTS; i++) {
		pa_write_reg(0, PA_REG_MAILBOX_SLOT(pdsp, i));
	}

	/* Enable PDSP */
	pa_write_reg(PA_REG_VAL_PDSP_CTL_ENABLE_PDSP(0),
		     PA_REG_PDSP_CTL(pdsp));

	return 0;
}
    
int keystone_pa_disable(int pdsp)
{
	u32 i, j;

	if (pdsp >= DEVICE_PA_NUM_PDSPS)
		return -EINVAL;

	if (pdsp < 0) {
		/* Disable all pdsps, clear all mailboxes */
		for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++)  {
			pa_write_reg(PA_REG_VAL_PDSP_CTL_DISABLE_PDSP,
				     PA_REG_PDSP_CTL(i));

			for (j = 0; j < PA_NUM_MAILBOX_SLOTS; j++)
				pa_write_reg(0, PA_REG_MAILBOX_SLOT(i, j));
		}
	} else {
		pa_write_reg(PA_REG_VAL_PDSP_CTL_DISABLE_PDSP,
			     PA_REG_PDSP_CTL(pdsp));

		for (j = 0; j < PA_NUM_MAILBOX_SLOTS; j++)
			pa_write_reg(0, PA_REG_MAILBOX_SLOT(pdsp, j));

	}
	return 0;
}

static int keystone_pa_config_timestamp(int factor)
{
	
	if (factor < PA_TIMESTAMP_SCALER_FACTOR_2 || 
		factor > PA_TIMESTAMP_SCALER_FACTOR_8192) 
		return -1;
	else {
		pa_write_reg(0xffff, PA_REG_TIMER_LOAD(0));
		pa_write_reg((PA_SS_TIMER_CNTRL_REG_GO |
			      PA_SS_TIMER_CNTRL_REG_MODE |
			      PA_SS_TIMER_CNTRL_REG_PSE |
			      (factor << PA_SS_TIMER_CNTRL_REG_PRESCALE_SHIFT)),
			     PA_REG_TIMER_CTL(0));
	}
	
	return 0;
}

int pa_get_timestamp(void)
{

	u32 timestamp;

	timestamp = 0x0000ffff - pa_read_reg(PA_REG_TIMER_VAL(0));
	timestamp += pa_read_reg(PA_MEM_PDSP_SRAM(PAFRM_SYS_TIMESTAMP_SRAM_INDEX) +
				 PAFRM_SYS_TIMESTAMP_OFFSET);

	return timestamp;
}

int pa_pdsp_run(int pdsp)
{
	int i, v;

	/* Check for enabled PDSP */
	v = pa_read_reg(PA_REG_PDSP_CTL(pdsp));
	if ((v & PA_REG_VAL_PDSP_CTL_ENABLE) ==
			PA_REG_VAL_PDSP_CTL_ENABLE) {
		/* Already enabled */
		return (PA_PDSP_ALREADY_ACTIVE);
	}

	/* Clear the mailbox */
	pa_write_reg(0, PA_REG_MAILBOX_SLOT(pdsp, 0));

	/* Set PDSP PC to 0, enable the PDSP */
	pa_write_reg(PA_REG_VAL_PDSP_CTL_ENABLE |
			PA_REG_VAL_PDSP_CTL_SOFT_RESET, 
			PA_REG_PDSP_CTL(pdsp));


	/* Wait for the mailbox to become non-zero */
	for (i = 0; i < PA_MAX_PDSP_ENABLE_LOOP_COUNT; i++)
		if (pa_read_reg(PA_REG_MAILBOX_SLOT(pdsp, 0)) != 0)
			return (PA_PDSP_RESET_RELEASED);

	return (PA_PDSP_NO_RESTART);
}

int keystone_pa_reset_control(int new_state)
{
	int i, res;
	int ret;
	int do_global_reset = 1;

	if (new_state == PA_STATE_ENABLE) {
		ret = PA_STATE_ENABLE;

		/*
		 * Do nothing if a pdsp is already out of reset.
		 * If any PDSPs are out of reset
		 * a global init is not performed
		 */
		for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
			res = pa_pdsp_run(i);
			
			if (res == PA_PDSP_ALREADY_ACTIVE)
				do_global_reset = 0;
    
			if (res == PA_PDSP_NO_RESTART) {
				ret = PA_STATE_ENABLE_FAILED;
				do_global_reset = 0;
			}
		}

		/* If global reset is required any PDSP can do it */
		if (do_global_reset) {
			pa_write_reg(1, PA_REG_MAILBOX_SLOT(0, 1)); /* Tell PDSP0 to do global init */
			pa_write_reg(0, PA_REG_MAILBOX_SLOT(0, 0)); /* Let PDSP0 go */
		
			while (pa_read_reg(PA_REG_MAILBOX_SLOT(0, 1)) != 0);  

			for (i = 1; i < DEVICE_PA_NUM_PDSPS; i++)
				pa_write_reg(0, PA_REG_MAILBOX_SLOT(i, 0)); /* Let PDSP0 go */
		} else
			for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++)
				pa_write_reg(0, PA_REG_MAILBOX_SLOT(i, 0)); /* Let PDSP0 go */
		
		return (ret);
	}
	return (PA_STATE_INVALID_REQUEST);
}

/*
 * download/upload firmware
 */
int keystone_pa_get_firmware(int pdsp, unsigned int *buffer, int len)
{
	if ((pdsp < 0) || (pdsp >= DEVICE_PA_NUM_PDSPS))
		return -EINVAL;

	pdsp_fw_get(buffer, (u32 *)(pa_base + PA_MEM_PDSP_IRAM(pdsp)),
		    len >> 2);

	return 0;
}

int keystone_pa_set_firmware(int pdsp, const unsigned int *buffer, int len)
{
	if ((pdsp < 0) || (pdsp >= DEVICE_PA_NUM_PDSPS))
		return -EINVAL;

	pdsp_fw_put((u32 *)(pa_base + PA_MEM_PDSP_IRAM(pdsp)), buffer,
		    len >> 2);

	/* Initialize the programmable constant registers C24-31 */
	pa_write_reg(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C25_C24],
		     PA_REG_PDSP_CONSTANT_TBL_BLOCK_INDEX0(pdsp));

	pa_write_reg(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C27_C26],
		     PA_REG_PDSP_CONSTANT_TBL_BLOCK_INDEX1(pdsp));

	pa_write_reg(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C29_C28],
		     PA_REG_PDSP_CONSTANT_TABLE_PROG_PTR_0(pdsp));

	pa_write_reg(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C31_C30],
		     PA_REG_PDSP_CONSTANT_TABLE_PROG_PTR_1(pdsp));

	return 0;
}

static struct pa_packet *pa_alloc_packet(struct pa_priv *priv,
					 unsigned cmd_size,
					 enum dma_data_direction direction)
{
	struct pa_packet *p_info;
	struct qm_host_desc *hd;
	u32 size;
	
       	size = cmd_size;
	if (size < L2_CACHE_BYTES)
		size = L2_CACHE_BYTES;

	p_info = kzalloc(sizeof(*p_info), GFP_KERNEL);
	if (!p_info)
		return NULL;

	p_info->priv      = priv;
	p_info->data      = kzalloc(size, GFP_KERNEL);
	if (!p_info->data) {
		kfree(p_info);
		return NULL;
	}
	p_info->direction = direction;
	p_info->queue     = (direction == DMA_TO_DEVICE) ? priv->tx_queue :
		priv->rx_queue;
	p_info->retqueue  = (direction == DMA_TO_DEVICE) ? priv->tx_retqueue :
		priv->rx_retqueue;
	p_info->cmd_size  =  cmd_size;
	
	hd = hw_qm_queue_pop(priv->free_queue);
	if (hd == NULL) {
		dev_err(priv->dev, "no more free desc hd = 0x%x\n", (u32) hd);
		return NULL;
	}

	hd->buff_len       = cmd_size;
	hd->orig_buff_len  = cmd_size;
	hd->orig_buff_ptr  = (u32) p_info->data;
	hd->buff_ptr       = (u32) p_info->data;

	p_info->desc = hd;

	QM_DESC_PINFO_SET_SIZE(hd->packet_info, 1);
	QM_DESC_DINFO_SET_PSINFO_LOC(hd->desc_info, 0);
	QM_DESC_DINFO_SET_PKT_LEN(hd->desc_info, cmd_size);
	QM_DESC_TINFO_SET_S_TAG_LO(hd->tag_info, 0);

	/* Set the return Queue */
	QM_DESC_PINFO_SET_QM(hd->packet_info, 0);
	QM_DESC_PINFO_SET_QUEUE(hd->packet_info, p_info->retqueue);

	return p_info;
}

static void pa_free_packet(struct pa_packet *p_info)
{
	if (p_info->data)
		kfree(p_info->data);
	kfree(p_info);
}

static int pa_submit_packet(struct pa_packet *p_info)
{
	struct qm_host_desc *hd = p_info->desc;

	if (p_info->direction == DMA_TO_DEVICE) {

		hd->software_info0 = p_info->swdata[0];
		hd->software_info1 = p_info->swdata[1];
		hd->software_info2 = p_info->swdata[2];
		hd->ps_data        = p_info->psdata[0];

		L2_cache_block_writeback((u32) p_info->data,
					 (u32) p_info->data + p_info->cmd_size);
	}
	
	hw_qm_queue_push(hd, p_info->queue, DEVICE_QM_DESC_SIZE_BYTES);

	return 0;
}

static void pa_complete_packet(struct pa_packet *p_info)
{
	struct qm_host_desc *hd;
	u32 loop = 0;

	/* Spin until we get free desc */
	while(((hd = hw_qm_queue_pop(p_info->retqueue)) == NULL)
	      && (++loop < 10000))
		udelay(10);

	if (loop == 10000) {
		printk(KERN_CRIT "PA: no answer receive from completion queue\n");
		return;
	}

	if (p_info->direction == DMA_FROM_DEVICE) {
		
		L2_cache_block_invalidate((u32) hd->orig_buff_ptr,
					  (u32) hd->orig_buff_ptr + hd->orig_buff_len);

		p_info->swdata[0] = hd->software_info0;
		p_info->swdata[1] = hd->software_info1;
		p_info->swdata[2] = hd->software_info2;
		p_info->psdata[0] = hd->ps_data;
	}
}

int keystone_pa_add_mac(struct pa_priv *priv, const u8 *mac, bool to_host,
			unsigned etype, int index)
{
	struct pa_frm_command *fcmd;
	struct pa_frm_cmd_add_lut1 *al1;
	struct pa_packet *tx, *rx;
	u32 context = 0x55550000;
	struct pa_route_info route_info, fail_info;
	int size, ret;
	
	memset(&fail_info, 0, sizeof(fail_info));

	fail_info.dest		= PA_DEST_HOST;
	fail_info.flow_id	= priv->data_flow_num;
	fail_info.queue		= priv->data_queue_num;
	fail_info.m_route_index	= -1;

	memset(&route_info, 0, sizeof(route_info));

	if (to_host) {
		route_info.dest			= PA_DEST_HOST;
		route_info.flow_id		= priv->data_flow_num;
		route_info.queue		= priv->data_queue_num;
		route_info.m_route_index	= -1;
	} else {
		route_info.dest			= PA_DEST_CONTINUE_PARSE_LUT1;
	}

	size = (sizeof(struct pa_frm_command) +
		sizeof(struct pa_frm_cmd_add_lut1) + 4);

	tx = pa_alloc_packet(priv, size, DMA_TO_DEVICE);
	if (!tx) {
		dev_err(priv->dev, "could not allocate cmd tx packet\n");
		return -ENOMEM;
	}

	rx = pa_alloc_packet(priv, size, DMA_FROM_DEVICE);
	if (!rx) {
		dev_err(priv->dev, "could not allocate cmd rx packet\n");
		pa_free_packet(tx);
		return -ENOMEM;
	}

	pa_submit_packet(rx);

	fcmd = tx->data;
	al1 = (struct pa_frm_cmd_add_lut1 *) &(fcmd->cmd);

	fcmd->command_result	= 0;
	fcmd->command		= PAFRM_CONFIG_COMMAND_ADDREP_LUT1;
	fcmd->magic		= PAFRM_CONFIG_COMMAND_SEC_BYTE;
	fcmd->com_id		= PA_COMID_L2;
	fcmd->ret_context	= context;
	fcmd->flow_id		= priv->cmd_flow_num;
	fcmd->reply_queue	= priv->cmd_queue_num;
	fcmd->reply_dest	= PAFRM_DEST_PKTDMA;

	al1->index		= index;
	al1->type		= PAFRM_COM_ADD_LUT1_STANDARD;

	if (etype) {
		al1->u.eth_ip.etype	   = etype;
		al1->u.eth_ip.match_flags |= PAFRM_LUT1_CUSTOM_MATCH_ETYPE;
	}
	
	al1->u.eth_ip.vlan	= 0;
	al1->u.eth_ip.pm.mpls	= 0;

	if (mac) {
		al1->u.eth_ip.dmac[0] = mac[0];
		al1->u.eth_ip.dmac[1] = mac[1];
		al1->u.eth_ip.dmac[2] = mac[2];
		al1->u.eth_ip.dmac[3] = mac[3];
		al1->u.eth_ip.dmac[4] = mac[4];
		al1->u.eth_ip.dmac[5] = mac[5];
		al1->u.eth_ip.key |= PAFRM_LUT1_KEY_MAC;
	}

	al1->u.eth_ip.smac[0] = 0;
	al1->u.eth_ip.smac[1] = 0;
	al1->u.eth_ip.smac[2] = 0;
	al1->u.eth_ip.smac[3] = 0;
	al1->u.eth_ip.smac[4] = 0;
	al1->u.eth_ip.smac[5] = 0;

	ret = pa_conv_routing_info(&al1->match, &route_info, 0, 0);
	if (ret != 0) 
		dev_err(priv->dev, "route info config failed\n");

	ret = pa_conv_routing_info(&al1->next_fail, &fail_info, 0, 1);
	if (ret != 0)
		dev_err(priv->dev, "fail info config failed\n");

	swizFcmd(fcmd);
	swizAl1((struct pa_frm_cmd_add_lut1 *)&(fcmd->cmd));

	tx->psdata[0] = ((u32)(4 << 5) << 24);

	tx->swdata[0] = 0x11112222;
	tx->swdata[1] = 0x33334444;
	tx->swdata[2] = 0;
	       
	pa_submit_packet(tx);
	dev_dbg(priv->dev, "waiting for command transmit complete\n");

	pa_complete_packet(tx);
	dev_dbg(priv->dev, "command transmit complete\n");

	dev_dbg(priv->dev, "waiting for command response complete\n");

	pa_complete_packet(rx);
	if (rx->swdata[0] != context) {
		dev_warn(priv->dev, "bad response context, have %x, want %x\n",
			 rx->swdata[0], context);
	} else {
		dev_dbg(priv->dev, "command response complete\n");
	}

	pa_free_packet(tx);
	pa_free_packet(rx);

	return 0;
}

/*
 * Used to configure the PA
 * dev: device associated to the PA
 * pdsp_data: platform data of the board
 * pdsp_num: number of PSDP to configure
 * rx_flow: NetCP receive flow
 * rx_queue: NetCP receive queue
 * free_queue: QMSS queue for free desc.
 */
int keystone_pa_config(struct device *dev,
		       struct pdsp_platform_data *pdsp_data,
		       int pdsp_num,
		       u8* mac_addr,
		       u32 rx_flow,
		       u32 rx_queue,
		       u32 free_queue)
{
	struct pa_priv      *priv;
	int                  ret = 0;
	int                  i;
	int                  factor;

	priv = kzalloc(sizeof(struct pa_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = dev;

	if ((pdsp_num < 0) || (pdsp_num > DEVICE_PA_NUM_PDSPS)) {
		ret = -EINVAL;
		goto fail;
	}

	if ((pdsp_data) && (pdsp_num > 0)) {

		/* Reset PDSP */
		ret = keystone_pa_reset();
		if (ret != 0) {
			dev_err(priv->dev, "PA reset failed\n");
			goto fail;
		}

		/* Load all requested firmware and enable PDSP */
		for (i = 0; i < pdsp_num; i++) {
			const struct firmware *fw = NULL;
			int pdsp = pdsp_data[i].pdsp;

			/* Request PA firmware */
			ret = request_firmware(&fw,
					       pdsp_data[i].firmware,
					       dev);
			if (ret != 0) {
				dev_err(priv->dev, "cannot find %s firmware\n",
					pdsp_data[i].firmware);
				goto fail;
			}

			/* Download the firmware in this PDSP */
			keystone_pa_set_firmware(pdsp, (const unsigned int*) fw->data , fw->size);

			release_firmware(fw);
		}
	}
	ret = keystone_pa_reset_control(PA_STATE_ENABLE);
	if (ret != 1) {
		dev_err(priv->dev, "enabling failed, ret = %d\n", ret);
		return ret;
	}

	factor = PA_TIMESTAMP_SCALER_FACTOR_2;

	ret = keystone_pa_config_timestamp(factor);
	if (ret != 0) {
		dev_err(priv->dev, "timestamp configuration failed, ret = %d\n", ret);
		return ret;
	}

	/* Configuration of flow/queues for PA command rx */
	priv->rx_queue       = DEVICE_QM_PA_CMD_FREE_Q;
	priv->rx_retqueue    = DEVICE_QM_PA_CMD_CP_Q;
	priv->cmd_flow_num   = DEVICE_PA_CDMA_RX_FIRMWARE_FLOW;
	priv->cmd_queue_num  = priv->rx_retqueue;

	/* Configuration of flow/queues for PA command tx */
	priv->tx_queue       = DEVICE_QM_PA_TX_PDSP0_Q;
	priv->tx_retqueue    = free_queue;

	/* Configuration of flow/queues for PA data rx */
	priv->data_queue_num = rx_queue;
	priv->data_flow_num  = rx_flow;
	priv->free_queue     = free_queue;

	dev_dbg(priv->dev, "configuring command receive flow %d, queue %d\n",
		rx_flow, rx_queue);

	ret = keystone_pa_add_mac(priv, NULL,     true,  0x0000,               63);
	ret = keystone_pa_add_mac(priv, mac_addr, false, PAFRM_ETHERTYPE_IP,   62);
	ret = keystone_pa_add_mac(priv, mac_addr, false, PAFRM_ETHERTYPE_IPV6, 61);
fail:
	kfree(priv);

	return ret;
}

static int __init keystone_pa_init(void)
{
	pa_base = ioremap(DEVICE_PA_BASE, DEVICE_PA_REGION_SIZE);
	if (!pa_base) {
		printk("Unable to map PA address register space\n");
		return -ENOMEM;
	}
	return 0;
}
subsys_initcall(keystone_pa_init); /* should be initialized early */
