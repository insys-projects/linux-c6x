/*
 *  linux/arch/c6x/platforms/include/mach/gemac.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Mark Salter <msalter@redhat.com>
 *          Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __MACH_C6X_GEMAC_H
#define __MACH_C6X_GEMAC_H

#include <asm/io.h>
#include <mach/board.h>

/*
 * Packet flags (16 bits)
 */
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

#define EMAC_PACKET_LEN_MASK      0x000007FF

/*
 * EMAC registers
 */
#define EMAC_TXIDVER              0x000 /* TX Identification and Version Register */
#define EMAC_TXCONTROL            0x004 /* TX Control Register */
#define EMAC_TXTEARDOWN           0x008 /* TX Teardown Register */
#define EMAC_RXIDVER              0x010 /* RX Identification and Version Register */
#define EMAC_RXCONTROL            0x014 /* RX Control Register */
#define EMAC_RXTEARDOWN           0x018 /* RX Teardown Register */
#define EMAC_TXINTSTATRAW         0x080 /* TX Interrupt Status Register (Unmasked) */
#define EMAC_TXINTSTATMASKED      0x084 /* TX Interrupt Status Register (Masked) */
#define EMAC_TXINTMASKSET         0x088 /* TX Interrupt Mask Set Register */
#define EMAC_TXINTMASKCLEAR       0x08C /* TX Interrupt Mask Clear Register */
#define EMAC_MACINVECTOR          0x090 /* MAC Input Vector Register */
#define EMAC_MACEOIVECTOR         0x094 /* MAC End of interrupt Vector Register */
#define EMAC_RXINTSTATRAW         0x0A0 /* RX Interrupt Status Register (Unmasked) */
#define EMAC_RXINTSTATMASKED      0x0A4 /* RX Interrupt Status Register (Masked) */
#define EMAC_RXINTMASKSET         0x0A8 /* RX Interrupt Mask Set Register */
#define EMAC_RXINTMASKCLEAR       0x0AC /* RX Interrupt Mask Clear Register */
#define EMAC_MACINTSTATRAW        0x0B0 /* MAC Interrupt Status Register (Unmasked) */
#define EMAC_MACINTSTATMASKED     0x0B4 /* MAC Interrupt Status Register (Masked) */
#define EMAC_MACINTMASKSET        0x0B8 /* MAC Interrupt Mask Set Register */
#define EMAC_MACINTMASKCLEAR      0x0BC /* MAC Interrupt Mask Clear Register */
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
#define EMAC_FIFOCONTROL          0x16C /* FIFO Control Register */
#define EMAC_MACCONFIG            0x170 /* MAC Configuration Register */
#define EMAC_SOFTRESET            0x174 /* Soft Reset Register */
#define EMAC_MACSRCADDRLO         0x1D0 /* MAC Source Address Low Bytes Register */
#define EMAC_MACSRCADDRHI         0x1D4 /* MAC Source Address High Bytes Register */
#define EMAC_MACHASH1             0x1D8 /* MAC Address Hash 1 Register */
#define EMAC_MACHASH2             0x1DC /* MAC Address Hash 2 Register */
#define EMAC_BOFFTEST             0x1E0 /* Backoff Test Register */
#define EMAC_TPACETEST            0x1E4 /* Transmit Pacing Test Register */
#define EMAC_RXPAUSE              0x1E8 /* Receive Pause Timer Register */
#define EMAC_TXPAUSE              0x1EC /* Transmit Pause Timer Register */
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
#define EMAC_MACADDRLO            0x500 /* MAC Address Low Bytes Register (rcv match) */
#define EMAC_MACADDRHI            0x504 /* MAC Address High Bytes Register (rcv match) */
#define EMAC_MACINDEX             0x508 /* MAC Index Register */
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


#if !(defined(CONFIG_SOC_TMS320C6474) || defined(CONFIG_SOC_TMS320C6457))

#if defined(CONFIG_SOC_TMS320C6472)
#define EMAC_EMIC_PID             0x1000
#define EMAC_PSCFG                0x1004
#define EMAC_RSVD0                0x1008
#define EMAC_EWINTCTL0            0x1020 /* Ethernet Wrapper: Interrupt Control Register */
#define EMAC_RSVD1                0x1034
#define EMAC_RPCFG0               0x10fc
#define EMAC_RPSTAT0              0x111c
#define EMAC_TPCFG0               0x113c
#define EMAC_TPSTAT0              0x115c

#define EMAC_B_HOST               (1 << 1)
#define EMAC_B_STAT               (1 << 2)
#define EMAC_B_MDIOLINT           (1 << 3)
#define EMAC_B_MDIOUSER           (1 << 4)
#define EMAC_B_TX0                (1 << 8)
#define EMAC_B_RX0                (1 << 16)

#else /* defined(CONFIG_SOC_TMS320C6472) */

#define EMAC_EWCTL                0x1004 /* Interrupt Control Register */
#define EMAC_EWINTTCNT            0x1008 /* Interrupt Timer Count */

#endif /* defined(CONFIG_SOC_TMS320C6472) */

#else /* !(defined(CONFIG_SOC_TMS320C6474) || defined(CONFIG_SOC_TMS320C6457)) */

#define ECTL_IDVER                0x00
#define ECTL_SOFTRESET            0x04
#define ECTL_EMCONTROL            0x08
#define ECTL_INTCONTROL           0x0c
#define ECTL_RXTHRESTEN           0x10
#define ECTL_RXEN                 0x14
#define ECTL_TXEN                 0x18
#define ECTL_MISCEN               0x1c

#define ECTL_RXTHRESTSTAT         0x40
#define ECTL_RXSTAT               0x44
#define ECTL_TXSTAT               0x48
#define ECTL_MISCSTAT             0x4c
#define ECTL_RXIMAX               0x70
#define ECTL_TXIMAX               0x74

#endif /* !(defined(CONFIG_SOC_TMS320C6474) || defined(CONFIG_SOC_TMS320C6457)) */

#define EMAC_NUM_STATREGS         36

/* Bit fields of EMAC registers */

#define EMAC_B_INTEN              (1 << 0)  /* for EWCTL */

#define EMAC_B_SOFTRST            (1 << 0)  /* for SOFTRESET */

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
#define EMAC_B_RXBUFERFLOWEN      (1 << 3)
#define EMAC_B_TXFLOWEN           (1 << 4)
#define EMAC_B_GMIIEN             (1 << 5)
#define EMAC_B_TXPACE             (1 << 6)
#define EMAC_B_GIG                (1 << 7)
#define EMAC_B_TXPTYPE            (1 << 9)
#define EMAC_B_CMDIDLE            (1 << 11)
#define EMAC_B_RXFIFOFLOWEN       (1 << 12)
#define EMAC_B_RXOWNERSHIP        (1 << 13)
#define EMAC_B_RXOFFLENBLOCK      (1 << 14)
#define EMAC_B_RMIISPEED          (1 << 15)
#define EMAC_B_RMIIDUPLEXMODE     (1 << 16)
#define EMAC_B_GIGFORCE           (1 << 17)
#define EMAC_B_RGMIIEN            (1 << 18) 
#define EMAC_B_CTL_EN             (1 << 18) 
#define EMAC_B_EXTEN              (1 << 18) /* for MACCONTROL */

#define EMAC_B_STATINT            (1 << 0)
#define EMAC_B_HOSTINT            (1 << 1)  /* for MACINTMASKSET */

#if defined(CONFIG_SOC_TMS320C6474) || defined(CONFIG_SOC_TMS320C6457)
#define EMAC_B_RXPEND0            (1 << 0) /* for MACINVECTOR */
#define EMAC_B_RXTHRESPEND0       (1 << 8)
#define EMAC_B_TXPEND0            (1 << 16)
#define EMAC_B_USERINT            (1 << 24)
#define EMAC_B_LINKINT            (1 << 25)
#define EMAC_B_HOSTPEND           (1 << 26)
#define EMAC_B_STATPEND           (1 << 27)
#else /* defined(CONFIG_SOC_TMS320C6474) */
#define EMAC_B_TXPEND0            (1 << 0)
#define EMAC_B_RXPEND0            (1 << 8)
#define EMAC_B_STATPEND           (1 << 16)
#define EMAC_B_HOSTPEND           (1 << 17) /* for MACINVECTOR */
#endif /* defined(CONFIG_SOC_TMS320C6474) || defined(CONFIG_SOC_TMS320C6457) */

#define EMAC_V_SOPERROR           1
#define EMAC_V_OWNERSHIP          2
#define EMAC_V_NOEOP              3
#define EMAC_V_NULLPTR            4
#define EMAC_V_NULLLEN            5
#define EMAC_V_LENRRROR           6
#define EMAC_S_RXERRCH            8
#define EMAC_S_TXERRCH            16
#define EMAC_S_RXERRCODE          12
#define EMAC_S_TXERRCODE          20
#define EMAC_M_RXERRCH            0x00000700
#define EMAC_M_TXERRCH            0x00070000
#define EMAC_M_RXERRCODE          0x0000f000
#define EMAC_M_TXERRCODE          0x00f00000 /* for MACSTATUS */

#define EMAC_S_VALID              20
#define EMAC_S_MATCHFILTER        19
#define EMAC_S_CHANNEL            16 /* for MACADDRLO */

/* Configuration mode flags */
#define EMAC_CONFIG_CHPRIORITY    0x01 /* use Tx channel priority */
#define EMAC_CONFIG_MACLOOPBACK   0x02 /* MAC internal loopback */
#define EMAC_CONFIG_RXCRC         0x04 /* include CRC in RX frames */
#define EMAC_CONFIG_TXCRC         0x08 /* Tx frames include CRC */
#define EMAC_CONFIG_PASSERROR     0x10 /* pass error frames */
#define EMAC_CONFIG_PASSCONTROL   0x20 /* pass control frames */
#define EMAC_CONFIG_PACING        0x40 /* use interrupt pacing */

#if defined(CONFIG_SGMII)

/*
 * SGMII registers
 */
#define SGMII_REG_BASE            0x02c40000
#define SGMII_IDVER_REG           (SGMII_REG_BASE + 0x000)
#define SGMII_SRESET_REG          (SGMII_REG_BASE + 0x004)
#define SGMII_CTL_REG             (SGMII_REG_BASE + 0x010)
#define SGMII_STATUS_REG          (SGMII_REG_BASE + 0x014)
#define SGMII_MRADV_REG           (SGMII_REG_BASE + 0x018)
#define SGMII_LPADV_REG           (SGMII_REG_BASE + 0x020)
#define SGMII_TXCFG_REG           (SGMII_REG_BASE + 0x030)
#define SGMII_RXCFG_REG           (SGMII_REG_BASE + 0x034)
#define SGMII_AUXCFG_REG          (SGMII_REG_BASE + 0x038)

#endif /* CONFIG_SGMII*/

#if defined(CONFIG_SOC_TMS320C6474) || defined(CONFIG_SOC_TMS320C6457)

/*
 * TCI6488/C6474/C6457 dependent definitions
 */ 
#define EMAC0_REG_BASE            0x02c80000
#define ECTL0_REG_BASE            0x02c81000
#define EMAC0_DSC_BASE            0x02c82000

#define EMAC_MAX_INSTANCE         1
#define PTR_TO_HW(x)              (x)
#define HW_TO_PTR(x)              (x)
#define _INTFLAG                  0
#define get_emac_idx()            (get_coreid())

#define emac_arch_get_hash1()     emac_get_reg(EMAC_MACHASH1)
#define emac_arch_get_hash2()     emac_get_reg(EMAC_MACHASH2)
#define emac_arch_get_promisc()   (emac_get_reg(EMAC_RXMBPENABLE) & EMAC_B_RXCAFEN)
#define emac_arch_get_control()   (emac_get_reg(EMAC_MACCONTROL) &	\
				       (EMAC_B_FULLDUPLEX | EMAC_B_RMIIDUPLEXMODE | \
					EMAC_B_GIG | EMAC_B_RMIISPEED))
#define emac_arch_set_hash1(a)    emac_set_reg(EMAC_MACHASH1, a)
#define emac_arch_set_hash2(a)    emac_set_reg(EMAC_MACHASH2, a)
#define emac_arch_set_promisc(a)  if (a) emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXCAFEN)
#define emac_arch_set_control(a)  if (a) emac_setbit_reg(EMAC_MACCONTROL, a)

#define emac_arch_init_control(pace)					\
	emac_setbit_reg(EMAC_MACCONTROL, (pace) ? EMAC_B_EXTEN | EMAC_B_GMIIEN | EMAC_B_TXPACE : \
			EMAC_B_EXTEN | EMAC_B_GMIIEN)

#define ectl_arch_rx_irq_enter(c) ectl_clearbit_reg(ECTL_RXEN + (get_coreid() << 4), \
						    1 << (c))
#define ectl_arch_rx_irq_leave(c) do {					\
		emac_set_reg(EMAC_MACEOIVECTOR, 0x1 + (get_coreid() << 2)); \
		ectl_setbit_reg(ECTL_RXEN + (get_coreid() << 4), 1 << (c)); \
	} while(0)
#define ectl_arch_tx_irq_enter(c) ectl_clearbit_reg(ECTL_TXEN + (get_coreid() << 4), \
						    1 << (c))
#define ectl_arch_tx_irq_leave(c) do {					\
		emac_set_reg(EMAC_MACEOIVECTOR, 0x2 + (get_coreid() << 2)); \
		ectl_setbit_reg(ECTL_TXEN + (get_coreid() << 4), 1 << (c)); \
	} while(0)

#define ectl_arch_enable_irq(c, misc) do {					\
		if (misc) ectl_set_reg(ECTL_MISCEN + (get_coreid() << 4), 0x3); \
		ectl_setbit_reg(ECTL_RXEN + (get_coreid() << 4), 1 << (c)); \
		ectl_setbit_reg(ECTL_TXEN + (get_coreid() << 4), 1 << (c)); \
	} while(0)

#define ectl_arch_disable_irq(c) do {					\
		ectl_set_reg(ECTL_MISCEN + (get_coreid() << 4), 0);	\
		ectl_clearbit_reg(ECTL_RXEN + (get_coreid() << 4), 1 << (c)); \
		ectl_clearbit_reg(ECTL_TXEN + (get_coreid() << 4), 1 << (c)); \
	} while(0)


#define emac_arch_get_int_vector()          emac_get_reg(EMAC_MACINVECTOR)
#define emac_arch_get_pending_irq(i, f, c)  ((i) & ((f) << (c)))
#define emac_arch_enable_irq(reg, c)        emac_set_reg(reg, 1 << (c))

#define emac_arch_reset_switch()

#define emac_arch_reset_ectl() do {			   \
		ectl_set_reg(ECTL_INTCONTROL, 0);	   \
		ectl_set_reg(ECTL_SOFTRESET, 1);	   \
		while (ectl_get_reg(ECTL_SOFTRESET) != 0); \
	} while(0)

#define emac_arch_set_pacing() do {					\
		ectl_set_reg(ECTL_INTCONTROL, gemac_int_prescaler());	\
		ectl_setbit_reg(ECTL_INTCONTROL, (0x3 << (get_coreid() << 1)) << 16); \
		ectl_set_reg(ECTL_RXIMAX + (get_coreid() << 3), 0x10);	\
		ectl_set_reg(ECTL_TXIMAX + (get_coreid() << 3), 0x10);	\
	} while(0)

#define emac_arch_clear_mac_addr()		 \
	for (i = 0; i < 32; i++) {		 \
		emac_set_reg(EMAC_MACINDEX, i);	 \
		emac_set_reg(EMAC_MACADDRHI, 0); \
		emac_set_reg(EMAC_MACADDRLO, 0); \
	}

#define emac_arch_set_mac_addr_index(i) emac_set_reg(EMAC_MACINDEX, i)
#define emac_arch_set_mac_addr_high(a)  emac_set_reg(EMAC_MACADDRHI, a)
#define emac_arch_set_mac_addr_low(a)   emac_set_reg(EMAC_MACADDRLO, a)
#define emac_arch_get_mac_addr_high(a)  emac_get_reg(EMAC_MACADDRHI)
#define emac_arch_get_mac_addr_low(a)   emac_get_reg(EMAC_MACADDRLO)

static inline int emac_check_shared_capability(void) 
{
#ifdef C6X_SOC_HAS_CORE_REV
	if (arch_get_silicon_rev() < 0x3)
		printk(KERN_WARNING "Warning sharing of GEMAC has issues with silicon rev %s!\n",
		       arch_compute_silicon_rev(arch_get_silicon_rev()));
#endif
	return 1;
}

#ifdef CONFIG_SOC_TMS320C6457
#include <asm/dscr.h>
#define EFUSE_REG_MAC_ADDR     DSCR_MACID1
#endif
#ifdef CONFIG_SOC_TMS320C6474
#define EFUSE_REG_MAC_ADDR     0x2880834
#endif

#define emac_arch_get_mac_addr emac_arch_get_mac_addr_from_efuse 

#define EMAC_HAS_SEPARATE_RXTX_IRQS
#define EMAC_ARCH_HAS_MAC_ADDR

#elif defined(CONFIG_SOC_TMS320C6472)

/*
 * TCI6486/C6472 dependent definitions
 */ 
#include <asm/dscr.h>

#define EMAC0_REG_BASE           0x02c80000
#define ECTL0_REG_BASE           0x02c81000
#define EMAC0_DSC_BASE           0x02c82000

#define EMAC1_REG_BASE           0x02cc0000
#define ECTL1_REG_BASE           0x02cc1000
#define EMAC1_DSC_BASE           0x02cc2000

#define EMAC_MAX_INSTANCE        2
#define PTR_TO_HW(x)             (x)
#define HW_TO_PTR(x)             (x)
#define _INTFLAG                 IRQF_DISABLED
#define get_emac_idx()           (get_coreid())

#define emac_arch_get_hash1()    emac_get_reg(EMAC_MACHASH1)
#define emac_arch_get_hash2()    emac_get_reg(EMAC_MACHASH2)
#define emac_arch_get_promisc()  (emac_get_reg(EMAC_RXMBPENABLE) & EMAC_B_RXCAFEN)
#define emac_arch_get_control()  emac_get_reg(EMAC_MACCONTROL)
#define emac_arch_set_hash1(a)   emac_set_reg(EMAC_MACHASH1, a)
#define emac_arch_set_hash2(a)   emac_set_reg(EMAC_MACHASH2, a)
#define emac_arch_set_promisc(a) if (a) emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXCAFEN)
#define emac_arch_set_control(a) if (a) emac_setbit_reg(EMAC_MACCONTROL, a)

#define emac_arch_init_control(pace) do {				\
		unsigned int dev_stat = dscr_get_reg(DSCR_DEVSTAT);	\
		unsigned int gigabit = 0, macsel;			\
		if(dev->dev_id == 0) {					\
			macsel = (dev_stat & DEVSTAT_B_EMAC0_MACSEL) >>	\
				DEVSTAT_B_EMAC0_OFFSET;			\
			if ((macsel == DEVSTAT_MACSEL_GMII) ||		\
			    (macsel == DEVSTAT_MACSEL_RGMII))		\
				gigabit = 1;				\
		}							\
		if(dev->dev_id == 1) {					\
			macsel = (dev_stat & DEVSTAT_B_EMAC1_MACSEL) >> \
				DEVSTAT_B_EMAC1_OFFSET;			\
			if (macsel == DEVSTAT_MACSEL_GMII)		\
				gigabit = 1;				\
		}							\
		if(gigabit == 1)					\
			emac_set_reg(EMAC_MACCONTROL, EMAC_B_GIG | EMAC_B_RGMIIEN); \
		else							\
			emac_set_reg(EMAC_MACCONTROL, EMAC_B_RMIISPEED); \
		emac_setbit_reg(EMAC_MACCONTROL, (pace) ?  EMAC_B_GMIIEN | EMAC_B_TXPACE: \
				EMAC_B_GMIIEN);				\
	} while(0)

#define ectl_arch_rx_irq_enter(c)
#define ectl_arch_rx_irq_leave(c)
#define ectl_arch_tx_irq_enter(c)
#define ectl_arch_tx_irq_leave(c)
#define ectl_arch_enable_irq(c, misc)					\
	if (misc) emac_set_reg(EMAC_EWINTCTL0 + (get_coreid() << 2),	\
			       EMAC_B_STAT | EMAC_B_HOST |		\
			       (EMAC_B_TX0 << (c)) | (EMAC_B_RX0 << (c))); \
	else emac_set_reg(EMAC_EWINTCTL0 + (get_coreid() << 2),		\
			  (EMAC_B_TX0 << (c)) | (EMAC_B_RX0 << (c)))
		
#define ectl_arch_disable_irq(c) \
	emac_set_reg(EMAC_EWINTCTL0 + (get_coreid() << 2), 0)

/* 
 * On Tomahawk, MACINVECTOR can be null even if TX/RX int occurs. 
 * This is probably an hardware bug.
 */
#define emac_arch_get_int_vector()          (emac_get_reg(EMAC_MACINVECTOR) \
					     | ((emac_get_reg(EMAC_RXINTSTATMASKED) & 0xff) << 8) \
					     | (emac_get_reg(EMAC_TXINTSTATMASKED) & 0xff))
#define emac_arch_get_pending_irq(i, f, c)  ((i) & ((f) << (c)))
#define emac_arch_enable_irq(reg, c)        emac_set_reg(reg, (1 << (c + 16)) | (1 << c))

#define emac_arch_reset_switch()

/* Disable EMAC/MDIO interrupts in EMIC */
#define emac_arch_reset_ectl()    emac_set_reg(EMAC_EWINTCTL0 + (get_coreid() << 2), 0x0)

#define emac_arch_set_pacing() do {					\
	        emac_set_reg(EMAC_RPCFG0 + (get_coreid() << 2), 0x3);   \
		emac_set_reg(EMAC_TPCFG0 + (get_coreid() << 2), 0x3);	\
		emac_set_reg(EMAC_PSCFG, gemac_int_prescaler());	\
	} while(0)

#define emac_arch_clear_mac_addr()		 \
	for (i = 0; i < 32; i++) {		 \
		emac_set_reg(EMAC_MACINDEX, i);	 \
		emac_set_reg(EMAC_MACADDRHI, 0); \
		emac_set_reg(EMAC_MACADDRLO, 0); \
	}

#define emac_arch_set_mac_addr_index(i) emac_set_reg(EMAC_MACINDEX, i)
#define emac_arch_set_mac_addr_high(a)  emac_set_reg(EMAC_MACADDRHI, a)
#define emac_arch_set_mac_addr_low(a)   emac_set_reg(EMAC_MACADDRLO, a)
#define emac_arch_get_mac_addr_high(a)  emac_get_reg(EMAC_MACADDRHI)
#define emac_arch_get_mac_addr_low(a)   emac_get_reg(EMAC_MACADDRLO)

#define emac_check_shared_capability()  1

#define EFUSE_REG_MAC_ADDR              0x2a80700 
#define emac_arch_get_mac_addr          emac_arch_get_mac_addr_from_efuse 

#define EMAC_HAS_SEPARATE_RXTX_IRQS
#define EMAC_ARCH_HAS_MAC_ADDR

#elif defined(CONFIG_SOC_TMS320C6455)

/*
 * C6455 dependent definitions
 */ 
#include <asm/dscr.h>

#define EMAC0_REG_BASE           0x02c80000
#define ECTL0_REG_BASE           0x02c81000
#define EMAC0_DSC_BASE           0x02c82000

#define EMAC_MAX_INSTANCE        1
#define PTR_TO_HW(x)             (x)
#define HW_TO_PTR(x)             (x)
#define _INTFLAG                 0
#define get_emac_idx()           0

#define emac_arch_get_hash1()    emac_get_reg(EMAC_MACHASH1)
#define emac_arch_get_hash2()    emac_get_reg(EMAC_MACHASH2)
#define emac_arch_get_promisc()  (emac_get_reg(EMAC_RXMBPENABLE) & EMAC_B_RXCAFEN)
#define emac_arch_get_control()  (emac_get_reg(EMAC_MACCONTROL) &	\
				      (EMAC_B_FULLDUPLEX | EMAC_B_RMIIDUPLEXMODE | \
				       EMAC_B_GIG | EMAC_B_RMIISPEED))
#define emac_arch_set_hash1(a)   emac_set_reg(EMAC_MACHASH1, a)
#define emac_arch_set_hash2(a)   emac_set_reg(EMAC_MACHASH2, a)
#define emac_arch_set_promisc(a) if (a) emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXCAFEN)
#define emac_arch_set_control(a) if (a) emac_setbit_reg(EMAC_MACCONTROL, a)
#define emac_arch_init_control(pace) \
	emac_setbit_reg(EMAC_MACCONTROL, (pace)? EMAC_B_GMIIEN | EMAC_B_TXPACE : EMAC_B_GMIIEN)

#define ectl_arch_rx_irq_enter(c)
#define ectl_arch_rx_irq_leave(c)
#define ectl_arch_tx_irq_enter(c)
#define ectl_arch_tx_irq_leave(c)
#define ectl_arch_enable_irq(c, misc)  emac_setbit_reg(EMAC_EWCTL, EMAC_B_INTEN)
#define ectl_arch_disable_irq(c) emac_clearbit_reg(EMAC_EWCTL, EMAC_B_INTEN)

#define emac_arch_get_int_vector()          emac_get_reg(EMAC_MACINVECTOR)
#define emac_arch_get_pending_irq(i, f, c)  ((i) & ((f) << (c)))
#define emac_arch_enable_irq(reg, c)        emac_set_reg(reg, 1 << (c))

#define emac_arch_reset_switch()

/* Disable EMAC/MDIO interrupts and reset EMAC/MDIO modules */
#define emac_arch_reset_ectl()    emac_clearbit_reg(EMAC_EWCTL, EMAC_B_INTEN)

#define emac_arch_set_pacing() do {					\
		/* Wait at least 100 cycles */				\
		emac_get_reg(EMAC_EWINTTCNT);				\
		emac_get_reg(EMAC_EWINTTCNT);				\
		emac_get_reg(EMAC_EWINTTCNT);				\
		emac_get_reg(EMAC_EWINTTCNT);				\
		emac_get_reg(EMAC_EWINTTCNT);				\
		/* Set interrupt timer count (decremented at cpuclk/6) */ \
		emac_set_reg(EMAC_EWINTTCNT, 15000);			\
	} while(0)

#define emac_arch_clear_mac_addr()		 \
	for (i = 0; i < 32; i++) {		 \
		emac_set_reg(EMAC_MACINDEX, i);	 \
		emac_set_reg(EMAC_MACADDRHI, 0); \
		emac_set_reg(EMAC_MACADDRLO, 0); \
	}

#define emac_arch_set_mac_addr_index(i) emac_set_reg(EMAC_MACINDEX, i)
#define emac_arch_set_mac_addr_high(a)  emac_set_reg(EMAC_MACADDRHI, a)
#define emac_arch_set_mac_addr_low(a)   emac_set_reg(EMAC_MACADDRLO, a)
#define emac_arch_get_mac_addr_high(a)  emac_get_reg(EMAC_MACADDRHI)
#define emac_arch_get_mac_addr_low(a)   emac_get_reg(EMAC_MACADDRLO)

/* Take RMII out of reset (the following 2 settings apply only to PG 2.0) */
#define rmii_arch_fix() do {						\
	if (intfmacsel == DEVSTAT_MACSEL_RMII)				\
	    dscr_set_reg(dscr_get_reg(DSCR_EMACCFG) |			\
			 DSCR_B_EMACCFG_RMIIRST, DSCR_EMACCFG);		\
    } while(0)

#define emac_check_shared_capability()  0

#define EMAC_TIMER_TICK_MDIO
#define EMAC_DO_INIT_MDIO

#else

/*
 * Generic definitions
 */

#define EMAC0_REG_BASE           0x02c80000
#define ECTL0_REG_BASE           0x02c81000
#define EMAC0_DSC_BASE           0x02c82000

#define EMAC_MAX_INSTANCE        1
#define PTR_TO_HW(x)             (x)
#define HW_TO_PTR(x)             (x)
#define _INTFLAG                 0
#define get_emac_idx()           0

#define emac_arch_get_hash1()    emac_get_reg(EMAC_MACHASH1)
#define emac_arch_get_hash2()    emac_get_reg(EMAC_MACHASH2)
#define emac_arch_get_promisc()  (emac_get_reg(EMAC_RXMBPENABLE) & EMAC_B_RXCAFEN)
#define emac_arch_get_control()  (emac_get_reg(EMAC_MACCONTROL) &  \
				      (EMAC_B_FULLDUPLEX | EMAC_B_RMIIDUPLEXMODE | \
				      EMAC_B_GIG | EMAC_B_RMIISPEED))
#define emac_arch_set_hash1(a)   emac_set_reg(EMAC_MACHASH1, a)
#define emac_arch_set_hash2(a)   emac_set_reg(EMAC_MACHASH2, a)
#define emac_arch_set_promisc(a) if (a) emac_setbit_reg(EMAC_RXMBPENABLE, EMAC_B_RXCAFEN)
#define emac_arch_set_control(a) if (a) emac_setbit_reg(EMAC_MACCONTROL, a)
#define emac_arch_init_control(pace) \
	emac_setbit_reg(EMAC_MACCONTROL, (pace)? EMAC_B_GMIIEN | EMAC_B_TXPACE : EMAC_B_GMIIEN)

#define ectl_arch_rx_irq_enter(c)
#define ectl_arch_rx_irq_leave(c)
#define ectl_arch_tx_irq_enter(c)
#define ectl_arch_tx_irq_leave(c)
#define ectl_arch_enable_irq(c, misc)  emac_setbit_reg(EMAC_EWCTL, EMAC_B_INTEN)
#define ectl_arch_disable_irq(c) emac_clearbit_reg(EMAC_EWCTL, EMAC_B_INTEN)

#define emac_arch_get_int_vector()          emac_get_reg(EMAC_MACINVECTOR)
#define emac_arch_get_pending_irq(i, f, c)  ((i) & ((f) << (c)))
#define emac_arch_enable_irq(reg, c)        emac_set_reg(reg, 1 << (c))

#define emac_arch_reset_switch()

/* Disable EMAC/MDIO interrupts and reset EMAC/MDIO modules */
#define emac_arch_reset_ectl()    emac_clearbit_reg(EMAC_EWCTL, EMAC_B_INTEN)

#define emac_arch_set_pacing() do {					\
		/* Wait at least 100 cycles */				\
		emac_get_reg(EMAC_EWINTTCNT);				\
		emac_get_reg(EMAC_EWINTTCNT);				\
		emac_get_reg(EMAC_EWINTTCNT);				\
		emac_get_reg(EMAC_EWINTTCNT);				\
		emac_get_reg(EMAC_EWINTTCNT);				\
		/* Set interrupt timer count (decremented at cpuclk/6) */ \
		emac_set_reg(EMAC_EWINTTCNT, 15000);			\
	} while(0)

#define emac_arch_clear_mac_addr()		 \
	for (i = 0; i < 32; i++) {		 \
		emac_set_reg(EMAC_MACINDEX, i);	 \
		emac_set_reg(EMAC_MACADDRHI, 0); \
		emac_set_reg(EMAC_MACADDRLO, 0); \
	}

#define emac_arch_set_mac_addr_index(i) emac_set_reg(EMAC_MACINDEX, i)
#define emac_arch_set_mac_addr_high(a)  emac_set_reg(EMAC_MACADDRHI, a)
#define emac_arch_set_mac_addr_low(a)   emac_set_reg(EMAC_MACADDRLO, a)
#define emac_arch_get_mac_addr_high(a)  emac_get_reg(EMAC_MACADDRHI)
#define emac_arch_get_mac_addr_low(a)   emac_get_reg(EMAC_MACADDRLO)

#define emac_check_shared_capability()  0

#define rmii_arch_fix() 

#define EMAC_TIMER_TICK_MDIO
#define EMAC_DO_INIT_MDIO

#endif

#ifdef EFUSE_REG_MAC_ADDR
/* Read the e-fuse value as 32 bit values to be endian independent */
static int inline emac_arch_get_mac_addr_from_efuse(char *x)
{
	unsigned int addr0, addr1;

	addr1 = __raw_readl(EFUSE_REG_MAC_ADDR + 4);
	addr0 = __raw_readl(EFUSE_REG_MAC_ADDR);

#if defined(CONFIG_SOC_TMS320C6472)
	x[0] = (addr0 & 0xff000000) >> 24;
	x[1] = (addr0 & 0x00ff0000) >> 16;
	x[2] = (addr0 & 0x0000ff00) >> 8;
	x[3] = addr0 & 0x000000ff;
	x[4] = (addr1 & 0xff000000) >> 24;
	x[5] = (addr1 & 0x00ff0000) >> 16;
#elif defined(CONFIG_SOC_TMS320C6474) || defined(CONFIG_SOC_TMS320C6457)
	x[0] = (addr1 & 0x0000ff00) >> 8;
	x[1] = addr1 & 0x000000ff;
	x[2] = (addr0 & 0xff000000) >> 24;
	x[3] = (addr0 & 0x00ff0000) >> 16;
	x[4] = (addr0 & 0x0000ff00) >> 8;
	x[5] = addr0 & 0x000000ff;
#else
#error "Missing SoC support for emac_arch_get_mac_addr"
#endif
	return 0;
}
#endif /* EFUSE_REG_MAC_ADDR */

#endif /* __MACH_C6X_GEMAC_H */
