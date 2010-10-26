/*
 *  linux/include/asm-c6x/gemac.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2006, 2009, 2010 Texas Instruments Incorporated
 *  Author: Nicolas Videau <nicolas.videau@jaluna.com>
 *          Aurelien Jacquiot <a-jacquiot@ti.com>
 *          
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_GEMAC_H_
#define __ASM_C6X_GEMAC_H_

#ifdef __KERNEL__

/*
 * Tx and Rx buffers parameters (total should be lower than 512)
 */
#define QUEUE_NUM         8                      /* Number of queues in the EMAC */
#define DESC_NUM          512                    /* Max number of descriptors in the descriptor 
						    memory (invariant) */
#define QUEUE_DESC_NUM    (DESC_NUM / QUEUE_NUM) /* Number of descriptors per queue */
#define MAC_ADDR_NUM      32                     /* Number of MAC addresses */

#define TX_RING_SIZE      (QUEUE_DESC_NUM / 2)   /* Must be a power of 2 */
#define RX_RING_SIZE      (QUEUE_DESC_NUM - TX_RING_SIZE)
#define RX_RING_MOD_MAX   (RX_RING_SIZE - 1)
#define TX_RING_MOD_MASK  (TX_RING_SIZE - 1)

#define PKT_MTU_CRC       ETH_FRAME_LEN + 4
#define PKT_MTU_NOCRC     ETH_FRAME_LEN

#define TX_TIMEOUT        (2*HZ)
#define EMAC_TIMER_PERIOD (HZ/10)

#define GEMAC_RESET_COLD  1
#define GEMAC_RESET_WARM  2

#define IDX_TO_CHAN(i)    ((i) << 1)              /* Convert index to channel */
#define IDX_TO_MAC(i)     ((i) << 1)              /* Convert index to MAC addr offset */
#define DEV_TO_MAC(d)     ((d) * CORE_NUM)        /* Convert dev_id to MAC addr offset */

/*
 * EMAC descriptor
 */
struct emac_desc {
	struct emac_desc  *next;
	u8                *buff;
	u32               buff_offset_len;
        u32               packet_flags_len;
};

/*
 * Private EMAC information
 */
struct emac_private {
	struct sk_buff *tx_skbuff[TX_RING_SIZE]; /* software TX skbuff ring */
	struct sk_buff *rx_skbuff[RX_RING_SIZE]; /* software RX skbuff ring */
	unsigned long skb_cur;
	unsigned long skb_tx_dirty;
	unsigned long skb_rx_dirty;
	unsigned long count_tx;

	struct emac_desc *rx_desc_base;
	struct emac_desc *tx_desc_base; /* address of rx and tx buffers base */
	struct emac_desc *cur_rx;
	struct emac_desc *cur_tx;	/* the next free ring entries */
	struct emac_desc *dirty_tx;	/* the ring entry to be freed */
	struct emac_desc *head_tx;      /* the new list head to be sent */

	struct net_device *dev;
	struct net_device_stats stats;
	unsigned long mode_flags;
	unsigned long slave;
	unsigned long packet_mtu;
	unsigned long tx_full;
	unsigned long fatal_error;
	spinlock_t    lock;
	unsigned long emac_reg_base;
	unsigned long ectl_reg_base;
	unsigned long emac_dsc_base;
	unsigned long mdio_reg_base;
#ifdef EMAC_HAS_ALE_SUPPORT
	unsigned int  mcast_valid_len;
	unsigned int *mcast_infos;
#endif
};

/*
 * EMAC Configuration information
 */
struct emac_config {
	u32 flags;
	u32 enetaddr[6];
};

#define emac_setbit_reg(reg, val)					\
        *((volatile u32 *) (ep->emac_reg_base + (reg))) |= (u32) (val)

#define emac_clearbit_reg(reg, val)					\
        *((volatile u32 *) (ep->emac_reg_base + (reg))) &= ~((u32) (val))
        
#define emac_set_reg(reg, val)						\
        *((volatile u32 *) (ep->emac_reg_base + (reg))) = (u32) (val)
        
#define emac_get_reg(reg)				\
        *((volatile u32 *) (ep->emac_reg_base + (reg)))

#define emac_addr_reg(reg)				\
        ((volatile u32 *) (ep->emac_reg_base + (reg)))

#define emac_set_stat(w, reg)			\
        do {					\
                u32 stat = emac_get_reg(reg);	\
                emac_set_reg(reg, stat);	\
                stat += (w);			\
                (w) = stat;			\
        } while(0)

#if defined(CONFIG_SOC_TMS320C6457) || defined(CONFIG_SOC_TMS320C6472) || defined(CONFIG_SOC_TMS320C6474)
#define ectl_setbit_reg(reg, val)					\
        *((volatile u32 *) (ep->ectl_reg_base + (reg))) |= (u32) (val)
	    
#define ectl_clearbit_reg(reg, val)					\
        *((volatile u32 *) (ep->ectl_reg_base + (reg))) &= ~((u32) (val))
        
#define ectl_set_reg(reg, val)						\
        *((volatile u32 *) (ep->ectl_reg_base + (reg))) = (u32) (val)
        
#define ectl_get_reg(reg)				\
        *((volatile u32 *) (ep->ectl_reg_base + (reg)))

/* Value for interrupt pacing: (CPUCLK/6) / 250000 (1/4us) = 0x29a on 1GHz DSP */
#define gemac_int_prescaler()					\
        (((CONFIG_TMS320C6X_MHZ * (1000000 / 6)) / 250000))

#endif /* defined(CONFIG_SOC_TMS320C6457) || defined(CONFIG_SOC_TMS320C6472) || defined(CONFIG_SOC_TMS320C6474) */

#include <mach/gemac.h>

#endif /* __KERNEL__ */
#endif /* __ASM_C6X_EMAC_H_ */
