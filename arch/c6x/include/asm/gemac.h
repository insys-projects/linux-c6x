/*
 *  linux/include/asm-c6x/gemac.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2006, 2009, 2010 Texas Instruments Incorporated
 *  Author: Nicolas Videau (nicolas.videau@jaluna.com)
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
#define RX_RING_SIZE      446
#define TX_RING_SIZE      64 /* Must be 2^X */
#define RX_RING_MOD_MAX   (RX_RING_SIZE - 1)
#define TX_RING_MOD_MASK  (TX_RING_SIZE - 1)

#define PKT_MTU_CRC       ETH_FRAME_LEN + 4
#define PKT_MTU_NOCRC     ETH_FRAME_LEN

#define TX_TIMEOUT        (2*HZ)
#define EMAC_TIMER_PERIOD (HZ/10)

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
	unsigned long packet_mtu;
	unsigned long tx_full;
	unsigned long zero_copy;
	unsigned long fatal_error;
	spinlock_t lock;
	unsigned long emac_reg_base;
	unsigned long ectl_reg_base;
	unsigned long emac_dsc_base;
	unsigned long mdio_reg_base;
#ifdef CONFIG_TMS320DM648
	unsigned int mcast_valid_len;
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


#define emac_setbit_reg(reg, val) \
        *((volatile u32 *) (ep->emac_reg_base + (reg))) |= (u32) (val)
	    
#define emac_clearbit_reg(reg, val) \
        *((volatile u32 *) (ep->emac_reg_base + (reg))) &= ~((u32) (val))
        
#define emac_set_reg(reg, val) \
        *((volatile u32 *) (ep->emac_reg_base + (reg))) = (u32) (val)
        
#define emac_get_reg(reg) \
        *((volatile u32 *) (ep->emac_reg_base + (reg)))

#define emac_addr_reg(reg) \
        ((volatile u32 *) (ep->emac_reg_base + (reg)))

#define emac_set_stat(w, reg) \
        do { \
                u32 stat = emac_get_reg(reg); \
                emac_set_reg(reg, stat); \
                stat += (w); \
                (w) = stat; \
        } while(0)

#if defined(CONFIG_SOC_TMS320C6457) || defined(CONFIG_SOC_TMS320C6472) || defined(CONFIG_SOC_TMS320C6474) || defined(CONFIG_TMS320DM648)
#define ectl_setbit_reg(reg, val) \
        *((volatile u32 *) (ep->ectl_reg_base + (reg))) |= (u32) (val)
	    
#define ectl_clearbit_reg(reg, val) \
        *((volatile u32 *) (ep->ectl_reg_base + (reg))) &= ~((u32) (val))
        
#define ectl_set_reg(reg, val) \
        *((volatile u32 *) (ep->ectl_reg_base + (reg))) = (u32) (val)
        
#define ectl_get_reg(reg) \
        *((volatile u32 *) (ep->ectl_reg_base + (reg)))

/* Value for interrupt pacing: (CPUCLK/6) / 250000 (1/4us) = 0x29a on 1GHz DSP */
#ifndef CONFIG_NK
#define gemac_int_prescaler() \
        (((CONFIG_TMS320C6X_MHZ * (1000000 / 6)) / 250000))
#else /* CONFIG_NK */
#define gemac_int_prescaler() \
        (nkctx->boot_info->clocksrc ? \
         ((nkctx->boot_info->clocksrc / 6) / 250000) : \
	 ((CONFIG_TMS320C6X_MHZ * (1000000 / 6)) / 250000) \
        )
#endif /* CONFIG_NK */
#endif /* defined(CONFIG_SOC_TMS320C6472) || defined(CONFIG_TMS320DM648) */

#include <mach/gemac.h>

#endif /* __KERNEL__ */
#endif /* __ASM_C6X_EMAC_H_ */
