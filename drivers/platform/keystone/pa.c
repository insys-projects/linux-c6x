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

int pa_config_timestamp(int factor)
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
		for (i = 0; i < 6; i++) {
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

			for (i = 1; i < 6; i++)
				pa_write_reg(0, PA_REG_MAILBOX_SLOT(i, 0)); /* Let PDSP0 go */
		} else
			for (i = 0; i < 6; i++)
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
	u32 const_reg_map = 0x007F;

	if ((pdsp < 0) || (pdsp >= DEVICE_PA_NUM_PDSPS))
		return -EINVAL;
	
	pdsp_fw_put((u32 *)(pa_base + PA_MEM_PDSP_IRAM(pdsp)), buffer,
		    len >> 2);
	
	if (pdsp == 4)
		const_reg_map |= (0x00700000);  
	else if (pdsp == 5)
		const_reg_map |= (0x00710000);
	
	pa_write_reg(const_reg_map,
		     PA_REG_PDSP_CONSTANT_TBL_BLOCK_INDEX0(pdsp)); 
	
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
	struct qm_host_desc        *hd, *hd1;
	int                         ret = 0;	
	int	                    i, factor;
	int                         size;
	const struct firmware      *fw;
	struct pa_frm_command      *fcmd;
	struct pa_frm_cmd_add_lut1 *al1;
	u32                         ps_cmd = ((u32)(4 << 5) << 24);

	struct pa_route_info	route_info  = {PA_DEST_HOST, rx_flow,
					       rx_queue, -1, 0, 0, 0, 0, 0, NULL};
	struct pa_route_info    n_fail_info = {PA_DEST_HOST, rx_flow,
					       rx_queue, -1, 0, 0, 0, 0, 0, NULL};
	struct pa_priv         *priv;
	
	priv = kzalloc(sizeof(struct pa_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = dev;
	
	printk(KERN_DEBUG "%s: using rx_flow %d, rx_queue = %d\n", __FUNCTION__,
	       rx_flow, rx_queue);

	/* Reset PDSP */
	ret = keystone_pa_reset();
	if (ret != 0) {
		printk(KERN_CRIT "%s: PA reset failed d\n", __FUNCTION__);
		return ret;
	}

	for (i = 0; i < pdsp_num; i++) {
       
		ret = request_firmware(&fw, 
				       pdsp_data[i].firmware,
				       dev);
		if (ret != 0) {
			printk(KERN_CRIT "PA: Cannot find  %s firmware for PDSP %d\n",
			       pdsp_data[i].firmware,
			       pdsp_data[i].pdsp);
			return ret;
		}
		
		/* Download the firmware to the PDSP */
		keystone_pa_set_firmware(pdsp_data[i].pdsp,
					 (const unsigned int*) fw->data, fw->size);

		release_firmware(fw);
	}
	
	ret = keystone_pa_reset_control(PA_STATE_ENABLE);
	if (ret != 1) {
		printk(KERN_CRIT "PA: enabling failed ret = %d\n", ret);
		return ret;
	}

	factor = PA_TIMESTAMP_SCALER_FACTOR_2;

	ret = pa_config_timestamp(factor);
	if (ret != 0) {
		printk(KERN_CRIT "PA: Timestamp Configuration failed ret = %d\n", ret);
		return ret;
	}
	
	priv->data_queue_num = rx_queue;
	priv->data_flow_num  = rx_flow;
	priv->free_queue     = free_queue;               /* free queue for command */
	priv->cmd_queue_num  = DEVICE_QM_PA_CMD_TX_CP_Q; /* return queue for pktdma */
	
	/* Form the configuration command in a buffer linked to a descriptor */
	hd = hw_qm_queue_pop(priv->free_queue);
	if (hd == NULL) {
		printk(KERN_CRIT "PA: no more free desc hd = 0x%x\n", (u32) hd);
		return -ENOMEM;
	}
	
	size = (sizeof(struct pa_frm_command) +
		sizeof(struct pa_frm_cmd_add_lut1)) + 4;
	
	fcmd = (struct pa_frm_command *) kzalloc(max(size, L2_CACHE_BYTES), GFP_KERNEL);
	if (fcmd == NULL)
		return -ENOMEM;
	
	al1 = (struct pa_frm_cmd_add_lut1 *) &(fcmd->cmd);

	fcmd->command_result  = 0;
	fcmd->command	      = PAFRM_CONFIG_COMMAND_ADDREP_LUT1;
	fcmd->magic	      = PAFRM_CONFIG_COMMAND_SEC_BYTE;
	fcmd->com_id	      = PA_COMID_L2;
	fcmd->ret_context     = 0x55550000;	
	fcmd->reply_queue     = priv->cmd_queue_num;
	fcmd->flow_id	      = rx_flow;
	fcmd->reply_dest      = PAFRM_DEST_PKTDMA;

	al1->index	      = 63;
	al1->type	      = PAFRM_COM_ADD_LUT1_STANDARD;
	
	al1->u.eth_ip.etype   = 0;
	al1->u.eth_ip.vlan    = 0;
	al1->u.eth_ip.pm.mpls = 0;
	
	al1->u.eth_ip.dmac[0] = 0;  
	al1->u.eth_ip.dmac[1] = 0;
	al1->u.eth_ip.dmac[2] = 0;
	al1->u.eth_ip.dmac[3] = 0;
	al1->u.eth_ip.dmac[4] = 0;
	al1->u.eth_ip.dmac[5] = 0;

	al1->u.eth_ip.smac[0] = 0;  
	al1->u.eth_ip.smac[1] = 0;
	al1->u.eth_ip.smac[2] = 0;
	al1->u.eth_ip.smac[3] = 0;
	al1->u.eth_ip.smac[4] = 0;
	al1->u.eth_ip.smac[5] = 0;

	al1->u.eth_ip.match_flags |= PAFRM_LUT1_CUSTOM_MATCH_KEY;

	ret = pa_conv_routing_info(&al1->match, &route_info, 0, 0);
	if (ret != 0)
		printk(KERN_CRIT "Route Info config failed\n");

	ret = pa_conv_routing_info(&al1->next_fail, &n_fail_info, 0, 1);
	if (ret != 0)
		printk(KERN_CRIT "Fail Info config failed\n");

	swizFcmd(fcmd);
	swizAl1((struct pa_frm_cmd_add_lut1 *)&(fcmd->cmd));

	L2_cache_block_writeback((u32) fcmd,
				 (u32) fcmd + max(size, L2_CACHE_BYTES));

	/* Send the command to the PA through the QM */
	QM_DESC_PINFO_SET_SIZE(hd->packet_info, 1);
	hd->buff_len       = size;
	hd->orig_buff_len  = size;
	hd->orig_buff_ptr  = (u32)fcmd;
	hd->buff_ptr	   = (u32)fcmd;
	hd->software_info0 = 0x11112222;
	hd->software_info1 = 0x33334444;
	hd->software_info2 = 0;
	hd->ps_data	   = ps_cmd;
	QM_DESC_DINFO_SET_PKT_LEN(hd->desc_info, size);

	/* Set the return Queue */
	QM_DESC_PINFO_SET_QM(hd->packet_info, 0);
	QM_DESC_PINFO_SET_QUEUE(hd->packet_info, free_queue);

	hw_qm_queue_push(hd, DEVICE_QM_PA_TX_PDSP0_Q, DEVICE_QM_DESC_SIZE_BYTES);

	for (i = 0; i < 100; i++) {
		udelay(1000);

		hd1 = hw_qm_queue_pop(priv->cmd_queue_num);

		if (hd1->software_info0 != 0x55550000) {
			printk(KERN_CRIT "Add_MAC: Found an entry in with swinfo0 = 0x%x\n",
			       hd1->software_info0);
			hd1->buff_len = hd1->orig_buff_len;
			hw_qm_queue_push(hd1, priv->free_queue, DEVICE_QM_DESC_SIZE_BYTES);
			return -1;
		} else {
			hd1->buff_len = hd1->orig_buff_len;
			hw_qm_queue_push(hd1, priv->free_queue, DEVICE_QM_DESC_SIZE_BYTES);
			break;
		}
	}
	return 0;
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
