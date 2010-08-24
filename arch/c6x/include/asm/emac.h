/*
 *  linux/include/asm-c6x/emac.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_EMAC_H_
#define __ASM_C6X_EMAC_H_

#ifdef __KERNEL__
#include <asm/hardware.h>

/*
 * Tx and Rx buffers parameters
 */
#define RX_RING_SIZE      128
#define TX_RING_SIZE      16
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
	struct sk_buff *tx_skbuff[TX_RING_SIZE]; /* software skbuff ring */
	unsigned long skb_cur;
	unsigned long skb_dirty;
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
	unsigned long fatal_error;
	spinlock_t lock;
};


/*
 * EMAC Configuration information
 */
struct emac_config {
	u32 flags;
	u32 enetaddr[6];
};

/* Packet flags (16 bits) */
#define EMAC_DESC_FLAG_SOP        0x80000000
#define EMAC_DESC_FLAG_EOP        0x40000000
#define EMAC_DESC_FLAG_OWNER      0x20000000
#define EMAC_DESC_FLAG_EOQ        0x10000000
#define EMAC_DESC_FLAG_TDOWNCMPLT 0x08000000
#define EMAC_DESC_FLAG_PASSCRC    0x04000000
#define EMAC_DESC_FLAG_JABBER     0x02000000
#define EMAC_DESC_FLAG_OVERSIZE   0x01000000
#define EMAC_DESC_FLAG_FRAGMENT   0x00800000
#define EMAC_DESC_FLAG_UNDERSIZED 0x00400000
#define EMAC_DESC_FLAG_CONTROL    0x00200000
#define EMAC_DESC_FLAG_OVERRUN    0x00100000
#define EMAC_DESC_FLAG_CODEERROR  0x00080000
#define EMAC_DESC_FLAG_ALIGNERROR 0x00040000
#define EMAC_DESC_FLAG_CRCERROR   0x00020000
#define EMAC_DESC_FLAG_NOMATCH    0x00010000

/*
 * EMAC registers
 */
#define EMAC_TXIDVER              0x000 /* TX Identification and Version Register */
#define EMAC_TXCONTROL            0x004 /* TX Control Register */
#define EMAC_TXTEARDOWN           0x008 /* TX Teardown Register */
#define EMAC_RXIDVER              0x010 /* RX Identification and Version Register */
#define EMAC_RXCONTROL            0x014 /* RX Control Register */
#define EMAC_RXTEARDOWN           0x018 /* RX Teardown Register */
#define EMAC_RXMBPENABLE          0x100 /* RX Mulicast/Bcast/Promisc Channel Enable Register */
#define EMAC_RXUNICASTSET         0x104 /* RX Unicast Set Register */
#define EMAC_RXUNICASTCLEAR       0x108 /* RX Unicast Clear Register */
#define EMAC_RXMAXLEN             0x10c /* RX Maximum Length Register */
#define EMAC_RXBUFFEROFFSET       0x110 /* RX Buffer Offset Register */
#define EMAC_RXFILTERLOWTHRESH    0x114 /* RX Filer Low Priority Packets Threshhold */
#define EMAC_RX0FLOWTHRESH        0x120 /* RX Channel 0 Flow Control Threshhold */
#define EMAC_RX1FLOWTHRESH        0x124 /* RX Channel 1 Flow Control Threshhold */
#define EMAC_RX2FLOWTHRESH        0x128 /* RX Channel 2 Flow Control Threshhold */
#define EMAC_RX3FLOWTHRESH        0x12c /* RX Channel 3 Flow Control Threshhold */
#define EMAC_RX4FLOWTHRESH        0x130 /* RX Channel 4 Flow Control Threshhold */
#define EMAC_RX5FLOWTHRESH        0x134 /* RX Channel 5 Flow Control Threshhold */
#define EMAC_RX6FLOWTHRESH        0x138 /* RX Channel 6 Flow Control Threshhold */
#define EMAC_RX7FLOWTHRESH        0x13c /* RX Channel 7 Flow Control Threshhold */
#define EMAC_RX0FREEBUFFER        0x140 /* RX Channel 0 Free Buffer Count Register */
#define EMAC_RX1FREEBUFFER        0x144 /* RX Channel 1 Free Buffer Count Register */
#define EMAC_RX2FREEBUFFER        0x148 /* RX Channel 2 Free Buffer Count Register */
#define EMAC_RX3FREEBUFFER        0x14c /* RX Channel 3 Free Buffer Count Register */
#define EMAC_RX4FREEBUFFER        0x150 /* RX Channel 4 Free Buffer Count Register */
#define EMAC_RX5FREEBUFFER        0x154 /* RX Channel 5 Free Buffer Count Register */
#define EMAC_RX6FREEBUFFER        0x158 /* RX Channel 6 Free Buffer Count Register */
#define EMAC_RX7FREEBUFFER        0x15c /* RX Channel 7 Free Buffer Count Register */
#define EMAC_MACCONTROL           0x160 /* MAC Control Register */
#define EMAC_MACSTATUS            0x164 /* MAC Status Register */
#define EMAC_EMCONTROL            0x168 /* Emulation Control Register */
#define EMAC_TXINTSTATRAW         0x170 /* TX Interrupt Status Register (Unmasked) */
#define EMAC_TXINTSTATMASKED      0x174 /* TX Interrupt Status Register (Masked) */
#define EMAC_TXINTMASKSET         0x178 /* TX Interrupt Mask Set Register */
#define EMAC_TXINTMASKCLEAR       0x17c /* TX Interrupt Mask Clear Register */
#define EMAC_MACINVECTOR          0x180 /* MAC Input Vector */
#define EMAC_MACEOIVECTOR         0x184 /* MAC EOI Vector */
#define EMAC_RXINTSTATRAW         0x190 /* RX Interrupt Status Register (Unmasked) */
#define EMAC_RXINTSTATMASKED      0x194 /* RX Interrupt Status Register (Masked) */
#define EMAC_RXINTMASKSET         0x198 /* RX Interrupt Mask Set Register */
#define EMAC_RXINTMASKCLEAR       0x19c /* RX Interrupt Mask Clear Register */
#define EMAC_MACINTSTATRAW        0x1a0 /* MAC Interrupt Status Register (Unmasked) */
#define EMAC_MACINTSTATMASKED     0x1a4 /* MAC Interrupt Status Register (Masked) */
#define EMAC_MACINTMASKSET        0x1a8 /* MAC Interrupt Mask Set Register */
#define EMAC_MACINTMASKCLEAR      0x1ac /* MAC Interrupt Mask Clear Register */
#define EMAC_MACADDRL0            0x1b0 /* MAC Address Channel 0 Lower Byte Register */
#define EMAC_MACADDRL1            0x1b4 /* MAC Address Channel 1 Lower Byte Register */
#define EMAC_MACADDRL2            0x1b8 /* MAC Address Channel 2 Lower Byte Register */
#define EMAC_MACADDRL3            0x1bc /* MAC Address Channel 3 Lower Byte Register */
#define EMAC_MACADDRL4            0x1c0 /* MAC Address Channel 4 Lower Byte Register */
#define EMAC_MACADDRL5            0x1c4 /* MAC Address Channel 5 Lower Byte Register */
#define EMAC_MACADDRL6            0x1c8 /* MAC Address Channel 6 Lower Byte Register */
#define EMAC_MACADDRL7            0x1cc /* MAC Address Channel 7 Lower Byte Register */
#define EMAC_MACADDRM             0x1d0 /* MAC Address Middle Byte Register */
#define EMAC_MACADDRH             0x1d4 /* MAC Address High Bytes Register */
#define EMAC_MACHASH1             0x1d8 /* MAC Address Hash 1 Register */
#define EMAC_MACHASH2             0x1dc /* MAC Address Hash 2 Register */
#define EMAC_BOFFTEST             0x1e0 /* Backoff Test Register */
#define EMAC_TPACETEST            0x1e4 /* Transmit Pacing Test Register */
#define EMAC_RXPAUSE              0x1e8 /* Receive Pause Timer Register */
#define EMAC_TXPAUSE              0x1ec /* Transmit Pause Timer Register */
#define EMAC_RXGOODFRAMES         0x200 /* Number of Good Frames Received */
#define EMAC_RXBCASTFRAMES        0x204 /* Number of Good Broadcast Frames Received */
#define EMAC_RXMCASTFRAMES        0x208 /* Number of Good Multicast Frames Received */
#define EMAC_RXPAUSEFRAMES        0x20c /* Number of PauseRX Frames Received */
#define EMAC_RXCRCERRORS          0x210 /* Number of Frames Received with CRC Errors */
#define EMAC_RXALIGNCODEERRORS    0x214 /* Number of Frames Received with Alignment/Code Errors */
#define EMAC_RXOVERSIZED          0x218 /* Number of Oversized Frames Received */
#define EMAC_RXJABBER             0x21c /* Number of Jabber Frames Received */
#define EMAC_RXUNDERSIZED         0x220 /* Number of Undersized Frames Received */
#define EMAC_RXFRAGMENTS          0x224 /* Number of RX Frame Fragments Received */
#define EMAC_RXFILTERED           0x228 /* Number of RX Frames Filtered Based on Address */
#define EMAC_RXQOSFILTERED        0x22c /* Number of RX Frames Filtered Based on QoS Filtering */
#define EMAC_RXOCTETS             0x230 /* Total Number of Received Bytes in Good Frames */
#define EMAC_TXGOODFRAMES         0x234 /* Number of Good Frames Sent */
#define EMAC_TXBCASTFRAMES        0x238 /* Number of Good Broadcast Frames Sent */
#define EMAC_TXMCASTFRAMES        0x23c /* Number of Good Multicast Frames Sent */
#define EMAC_TXPAUSEFRAMES        0x240 /* Number of PauseTX Frames Sent */
#define EMAC_TXDEFERRED           0x244 /* Number of Frames Where Transmission was Deferred */
#define EMAC_TXCOLLISION          0x248 /* Total Number of Frames Sent That Experienced a Collision */
#define EMAC_TXSINGLECOLL         0x24c /* Number of Frames Sent with Exactly One Collision */
#define EMAC_TXMULTICOLL          0x250 /* Number of Frames Sent with Multiple Colisions */
#define EMAC_TXEXCESSIVECOLL      0x254 /* Number of TX Frames Lost Due to Excessive Collisions */
#define EMAC_TXLATECOLL           0x258 /* Number of TX Frames Lost Due to a Late Collision */
#define EMAC_TXUNDERRUN           0x25c /* Number of TX Frames Lost with Transmit Underrun Error */
#define EMAC_TXCARRIERSLOSS       0x260 /* Numebr of TX Frames Lost Due to Carrier Sense Loss */
#define EMAC_TXOCTETS             0x264 /* Total Nu,ber of Transmitted Bytes in Good Frames */
#define EMAC_FRAME64              0x268 /* Total TX & RX Frames with Octet Size of 64 */
#define EMAC_FRAME65T127          0x26c /* Total TX & RX Frames with Octet Size of 65 to 127 */
#define EMAC_FRAME128T255         0x270 /* Total TX & RX Frames with Octet Size of 128 to 255 */
#define EMAC_FRAME256T511         0x274 /* Total TX & RX Frames with Octet Size of 256 to 511 */
#define EMAC_FRAME512T1023        0x278 /* Total TX & RX Frames with Octet Size of 512 to 1023 */
#define EMAC_FRAME1024TUP         0x27c /* Total TX & RX Frames with Octet Size of 1024 or above */
#define EMAC_NETOCTETS            0x280 /* Sum of all Octets Sent or Received on the Network */
#define EMAC_RXSOFOVERRUNS        0x284 /* Total RX Start of Frame Overruns (FIFO or DMA) */
#define EMAC_RXMOFOVERRUNS        0x288 /* Total RX Middle of Frame Overruns (FIFO or DMA) */
#define EMAC_RXDMAOVERRUNS        0x28c /* Total RX DMA Overruns */
#define EMAC_TX0HDP               0x600 /* TX Channel 0 DMA Head Descriptor Pointer Register */
#define EMAC_TX1HDP               0x604 /* TX Channel 1 DMA Head Descriptor Pointer Register */
#define EMAC_TX2HDP               0x608 /* TX Channel 2 DMA Head Descriptor Pointer Register */
#define EMAC_TX3HDP               0x60c /* TX Channel 3 DMA Head Descriptor Pointer Register */
#define EMAC_TX4HDP               0x610 /* TX Channel 4 DMA Head Descriptor Pointer Register */
#define EMAC_TX5HDP               0x614 /* TX Channel 5 DMA Head Descriptor Pointer Register */
#define EMAC_TX6HDP               0x618 /* TX Channel 6 DMA Head Descriptor Pointer Register */
#define EMAC_TX7HDP               0x61c /* TX Channel 7 DMA Head Descriptor Pointer Register */
#define EMAC_RX0HDP               0x620 /* RX Channel 0 DMA Head Descriptor Pointer Register */
#define EMAC_RX1HDP               0x624 /* RX Channel 1 DMA Head Descriptor Pointer Register */
#define EMAC_RX2HDP               0x628 /* RX Channel 2 DMA Head Descriptor Pointer Register */
#define EMAC_RX3HDP               0x62c /* RX Channel 3 DMA Head Descriptor Pointer Register */
#define EMAC_RX4HDP               0x630 /* RX Channel 4 DMA Head Descriptor Pointer Register */
#define EMAC_RX5HDP               0x634 /* RX Channel 5 DMA Head Descriptor Pointer Register */
#define EMAC_RX6HDP               0x638 /* RX Channel 6 DMA Head Descriptor Pointer Register */
#define EMAC_RX7HDP               0x63c /* RX Channel 7 DMA Head Descriptor Pointer Register */
#define EMAC_TX0INTACK            0x640 /* TX Channel 0 Interrupt Acknowledge Register */
#define EMAC_TX1INTACK            0x644 /* TX Channel 1 Interrupt Acknowledge Register */
#define EMAC_TX2INTACK            0x648 /* TX Channel 2 Interrupt Acknowledge Register */
#define EMAC_TX3INTACK            0x64c /* TX Channel 3 Interrupt Acknowledge Register */
#define EMAC_TX4INTACK            0x650 /* TX Channel 4 Interrupt Acknowledge Register */
#define EMAC_TX5INTACK            0x654 /* TX Channel 5 Interrupt Acknowledge Register */
#define EMAC_TX6INTACK            0x658 /* TX Channel 6 Interrupt Acknowledge Register */
#define EMAC_TX7INTACK            0x65c /* TX Channel 7 Interrupt Acknowledge Register */
#define EMAC_RX0INTACK            0x660 /* RX Channel 0 Interrupt Acknowledge Register */
#define EMAC_RX1INTACK            0x664 /* RX Channel 1 Interrupt Acknowledge Register */
#define EMAC_RX2INTACK            0x668 /* RX Channel 2 Interrupt Acknowledge Register */
#define EMAC_RX3INTACK            0x66c /* RX Channel 3 Interrupt Acknowledge Register */
#define EMAC_RX4INTACK            0x670 /* RX Channel 4 Interrupt Acknowledge Register */
#define EMAC_RX5INTACK            0x674 /* RX Channel 5 Interrupt Acknowledge Register */
#define EMAC_RX6INTACK            0x678 /* RX Channel 6 Interrupt Acknowledge Register */
#define EMAC_RX7INTACK            0x67c /* RX Channel 7 Interrupt Acknowledge Register */
#define EMAC_EWTRCTRL             0x3000 /* TR Control */
#define EMAC_EWCTL                0x3004 /* Interrupt Control Register */
#define EMAC_EWINTTCNT            0x3008 /* Interrupt Timer Count */

#define EMAC_NUM_STATREGS         36

/* Bit fields of EMAC registers */

#define EMAC_B_INTEN              (1 << 0)
#define EMAC_B_MDIORST            (1 << 1)
#define EMAC_B_EMACRST            (1 << 2)  /* for EWCTL */

#define EMAC_B_MULTEN             (1 << 5)
#define EMAC_B_BROADEN            (1 << 13)
#define EMAC_B_RXCAFEN            (1 << 21)
#define EMAC_B_RXCEFEN            (1 << 22)
#define EMAC_B_RXCSFEN            (1 << 23)
#define EMAC_B_RXCMFEN            (1 << 24)
#define EMAC_B_RXNOCHAIN          (1 << 28)
#define EMAC_B_RXQOSEN            (1 << 29)
#define EMAC_B_RXPASSCRC          (1 << 30) /* for RXMBPENABLE */

#define EMAC_B_FULLDUPLEX         (1 << 0)
#define EMAC_B_LOOPBACK           (1 << 1)
#define EMAC_B_MTEST              (1 << 2)
#define EMAC_B_RXFLOWEN           (1 << 3)
#define EMAC_B_TXFLOWEN           (1 << 4)
#define EMAC_B_MIIEN              (1 << 5)
#define EMAC_B_TXPACE             (1 << 6)
#define EMAC_B_TXPTYPE            (1 << 9)  /* for MACCONTROL */

#define EMAC_B_STATINT            (1 << 0)
#define EMAC_B_HOSTERRINT         (1 << 1)  /* for MACINTMASKSET */

#define EMAC_B_TXPEND0            (1 << 0)
#define EMAC_B_RXPEND0            (1 << 8)
#define EMAC_B_STATPEND           (1 << 16)
#define EMAC_B_HOSTPEND           (1 << 17) /* for MACINVECTOR */

#define EMAC_V_SOPERROR           1
#define EMAC_V_OWNERSHIP          2
#define EMAC_V_NOEOP              3
#define EMAC_V_NULLPTR            4
#define EMAC_V_NULLLEN            5
#define EMAC_V_LENRRROR           6
#define EMAC_S_RXERRCODE          12
#define EMAC_S_TXERRCODE          20
#define EMAC_M_RXERRCODE          0x0f000
#define EMAC_M_TXERRCODE          0xf0000   /* for MACSTATUS */

/* Configuration mode flags */
#define EMAC_CONFIG_CHPRIORITY    0x01 /* use Tx channel priority */
#define EMAC_CONFIG_MACLOOPBACK   0x02 /* MAC internal loopback */
#define EMAC_CONFIG_RXCRC         0x04 /* include CRC in RX frames */
#define EMAC_CONFIG_TXCRC         0x08 /* Tx frames include CRC */
#define EMAC_CONFIG_PASSERROR     0x10 /* pass error frames */
#define EMAC_CONFIG_PASSCONTROL   0x20 /* pass control frames */

#define emac_setbit_reg(reg, val) \
        *((volatile u32 *) (EMAC_REG_BASE + (reg))) |= (u32) (val)
	    
#define emac_clearbit_reg(reg, val) \
        *((volatile u32 *) (EMAC_REG_BASE + (reg))) &= ~((u32) (val))
        
#define emac_set_reg(reg, val) \
        *((volatile u32 *) (EMAC_REG_BASE + (reg))) = (u32) (val)
        
#define emac_get_reg(reg) \
        *((volatile u32 *) (EMAC_REG_BASE + (reg)))

#define emac_addr_reg(reg) \
        ((volatile u32 *) (EMAC_REG_BASE + (reg)))

#define emac_set_stat(w, reg) \
        do { \
                u32 stat = emac_get_reg(reg); \
                emac_set_reg(reg, stat); \
                stat += (w); \
                (w) = stat; \
        } while(0)

#endif /* __KERNEL__ */
#endif /* __ASM_C6X_EMAC_H_ */
