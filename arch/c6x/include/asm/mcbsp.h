/*
 *  linux/include/asm-c6x/mcbsp.h
 *
 *  TI DAVINCI and TMS320C6x SoC McBSP driver Info
 *
 *  Copyright (C) 2006 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ******************************************************************************
 *
 * 2009-03-30   Aurelien Jacquiot / Nicolas Videau - Modified to support all
 *              TMS320C6x - Copyright (C) 2006, 2009, 2010 
 *              Texas Instruments Incorporated
 *
 ******************************************************************************
 */
#ifndef __ASM_ARCH_MCBSP_H
#define __ASM_ARCH_MCBSP_H

#include <asm/hardware.h>
#include <asm/irq.h>

#define MAX_MCBSP_COUNT       2

/* EDMA3 BUS TX and RX DATA */
#define MCBSP0_EDMA_RX_DATA   (MCBSP0_EDMA_BASE_ADDR)
#define MCBSP0_EDMA_TX_DATA   (MCBSP0_EDMA_BASE_ADDR + 0x10)
#define MCBSP1_EDMA_RX_DATA   (MCBSP1_EDMA_BASE_ADDR)
#define MCBSP1_EDMA_TX_DATA   (MCBSP1_EDMA_BASE_ADDR + 0x10)

#define McBSP0RX INT11
#define McBSP0TX INT14
#define McBSP1RX INT11
#define McBSP1TX INT14

#define DRR1	0x00
#define DRR2	0x02
#define DXR1	0x04
#define DXR2	0x06
#define SPCR1	0x08
#define SPCR2	0x0a
#define RCR1	0x0c
#define RCR2	0x0e
#define XCR1	0x10
#define XCR2	0x12
#define SRGR1	0x14
#define SRGR2	0x16
#define MCR1	0x18
#define MCR2	0x1a
#define RCERA	0x1c
#define RCERB	0x1e
#define XCERA	0x20
#define XCERB	0x22
#define PCR0	0x24
#define PCR1	0x26
#define RCERC	0x28
#define RCERD	0x2a
#define XCERC	0x2c
#define XCERD	0x2e
#define RCERE	0x30
#define RCERF	0x32
#define XCERE	0x34
#define XCERF	0x36
#define RCERG	0x38
#define RCERH	0x3a
#define XCERG	0x3c
#define XCERH	0x3e

/* 32bit acces definition */
#define DRR     0x00
#define DXR     0x04
#define SPCR    0x08
#define RCR     0x0c
#define XCR     0x10
#define SRGR    0x14
#define MCR     0x18
#define RCERE0  0x1c
#define XCERE0  0x20
#define PCR     0x24
#define RCERE1  0x28
#define XCERE1  0x2c
#define RCERE2  0x30
#define XCERE2  0x34
#define RCERE3  0x38
#define XCERE3  0x3c

/********************** McBSP SPCR1 bit definitions ***********************/
#define RRST		0x0001
#define RRDY		0x0002
#define RFULL		0x0004
#define RSYNC_ERR	0x0008
#define RINTM(value)	((value)<<4)	/* bits 4:5 */
#define ABIS		0x0040
#define DXENA		0x0080
#define CLKSTP(value)	((value)<<11)	/* bits 11:12 */
#define RJUST(value)	((value)<<13)	/* bits 13:14 */
#define DLB		0x8000

/********************** McBSP SPCR2 bit definitions ***********************/
#define XRST		0x0001
#define XRDY		0x0002
#define XEMPTY		0x0004
#define XSYNC_ERR	0x0008
#define XINTM(value)	((value)<<4)	/* bits 4:5 */
#define GRST		0x0040
#define FRST		0x0080
#define SOFT		0x0100
#define FREE		0x0200

/********************** McBSP PCR bit definitions *************************/
#define CLKRP		0x0001
#define CLKXP		0x0002
#define FSRP		0x0004
#define FSXP		0x0008
#define DR_STAT		0x0010
#define DX_STAT		0x0020
#define CLKS_STAT	0x0040
#define SCLKME		0x0080
#define CLKRM		0x0100
#define CLKXM		0x0200
#define FSRM		0x0400
#define FSXM		0x0800
#define RIOEN		0x1000
#define XIOEN		0x2000
#define IDLE_EN		0x4000

/********************** McBSP RCR1 bit definitions ************************/
#define RWDLEN1(value)		((value)<<5)	/* Bits 5:7 */
#define RFRLEN1(value)		((value)<<8)	/* Bits 8:14 */

/********************** McBSP XCR1 bit definitions ************************/
#define XWDREVRS                (1 << 4)
#define XWDLEN1(value)		((value)<<5)	/* Bits 5:7 */
#define XFRLEN1(value)		((value)<<8)	/* Bits 8:14 */

/*********************** McBSP RCR2 bit definitions ***********************/
#define RDATDLY(value)		(value)	/* Bits 0:1 */
#define RFIG			0x0004
#define RCOMPAND(value)		((value)<<3)	/* Bits 3:4 */
#define RWDLEN2(value)		((value)<<5)	/* Bits 5:7 */
#define RFRLEN2(value)		((value)<<8)	/* Bits 8:14 */
#define RPHASE			0x8000

/*********************** McBSP XCR2 bit definitions ***********************/
#define XDATDLY(value)		(value)	/* Bits 0:1 */
#define XFIG			0x0004
#define XCOMPAND(value)		((value)<<3)	/* Bits 3:4 */
#define XWDLEN2(value)		((value)<<5)	/* Bits 5:7 */
#define XFRLEN2(value)		((value)<<8)	/* Bits 8:14 */
#define XPHASE			0x8000

/********************* McBSP SRGR1 bit definitions ************************/
#define CLKGDV(value)		(value)	/* Bits 0:7 */
#define FWID(value)		((value)<<8)	/* Bits 8:15 */

/********************* McBSP SRGR2 bit definitions ************************/
#define FPER(value)		(value)	/* Bits 0:11 */
#define FSGM			0x1000
#define CLKSM			0x2000
#define CLKSP			0x4000
#define GSYNC			0x8000

/********************* McBSP MCR1 bit definitions *************************/
#define RMCM			0x0001
#define RCBLK(value)		((value)<<2)	/* Bits 2:4 */
#define RPABLK(value)		((value)<<5)	/* Bits 5:6 */
#define RPBBLK(value)		((value)<<7)	/* Bits 7:8 */

/********************* McBSP MCR2 bit definitions *************************/
#define XMCM(value)		(value)	/* Bits 0:1 */
#define XCBLK(value)		((value)<<2)	/* Bits 2:4 */
#define XPABLK(value)		((value)<<5)	/* Bits 5:6 */
#define XPBBLK(value)		((value)<<7)	/* Bits 7:8 */

/* we don't do multichannel for now */
struct mcbsp_reg_cfg {
	u16 spcr2;
	u16 spcr1;
	u16 rcr2;
	u16 rcr1;
	u16 xcr2;
	u16 xcr1;
	u16 srgr2;
	u16 srgr1;
	u16 mcr2;
	u16 mcr1;
	u16 pcr2;
	u16 pcr0;
	u16 rcerc;
	u16 rcerd;
	u16 xcerc;
	u16 xcerd;
	u16 rcere;
	u16 rcerf;
	u16 xcere;
	u16 xcerf;
	u16 rcerg;
	u16 rcerh;
	u16 xcerg;
	u16 xcerh;
};

typedef enum {
	MCBSP0 = 0,
	MCBSP1,
} mcbsp_id;

typedef enum {
	MCBSP_WORD_8 = 0,
	MCBSP_WORD_12,
	MCBSP_WORD_16,
	MCBSP_WORD_20,
	MCBSP_WORD_24,
	MCBSP_WORD_32,
} mcbsp_word_length;

typedef enum {
	MCBSP_CLK_RISING = 0,
	MCBSP_CLK_FALLING,
} mcbsp_clk_polarity;

typedef enum {
	MCBSP_FS_ACTIVE_HIGH = 0,
	MCBSP_FS_ACTIVE_LOW,
} mcbsp_fs_polarity;

typedef enum {
	MCBSP_CLK_STP_MODE_NO_DELAY = 0,
	MCBSP_CLK_STP_MODE_DELAY,
} mcbsp_clk_stp_mode;

typedef enum {
	MCBSP_MSB_FIRST = 0,
	MCBSP_LSB_FIRST,
} mcbsp_format_mode;

/******* SPI specific mode **********/
typedef enum {
	MCBSP_SPI_MASTER = 0,
	MCBSP_SPI_SLAVE,
} mcbsp_spi_mode;

struct mcbsp_spi_cfg {
	mcbsp_spi_mode spi_mode;
	mcbsp_clk_polarity rx_clock_polarity;
	mcbsp_clk_polarity tx_clock_polarity;
	mcbsp_fs_polarity fsx_polarity;
	u8 clk_div;
	mcbsp_clk_stp_mode clk_stp_mode;
	mcbsp_word_length word_length;
        mcbsp_format_mode format_mode;
};

struct mcbsp {
	struct device *dev;
	void __iomem *io_base;
	u8  id;
	u8  free;
	mcbsp_word_length rx_word_length;
	mcbsp_word_length tx_word_length;

	/* IRQ based TX/RX */
	int rx_irq;
	int tx_irq;

	/* DMA stuff */
	u8    dma_rx_sync;
	short dma_rx_lch;
	short dma_rx_lch_reload;
        u32   dma_rx_data;
	u8    dma_tx_sync;
	short dma_tx_lch;
	short dma_tx_lch_reload;
        u32   dma_tx_data;

	/* Completion queues */
	struct completion tx_irq_completion;
	struct completion rx_irq_completion;
	struct completion tx_dma_completion;
	struct completion rx_dma_completion;

	spinlock_t lock;
	struct mcbsp_info *pdata;
	struct clk        *clk;
};

extern struct mcbsp *mcbsp_ptr[MAX_MCBSP_COUNT];

void mcbsp_config(unsigned int id,
		  const struct mcbsp_reg_cfg *config);
int mcbsp_request(unsigned int id);
void mcbsp_free(unsigned int id);
void mcbsp_start(unsigned int id);
void mcbsp_start_raw(unsigned int id);
void mcbsp_start_tx(unsigned int id);
void mcbsp_start_rx(unsigned int id);
void mcbsp_stop(unsigned int id);
void mcbsp_stop_tx(unsigned int id);
void mcbsp_stop_rx(unsigned int id);
void mcbsp_xmit_word(unsigned int id, u32 word);
u32 mcbsp_recv_word(unsigned int id);

int mcbsp_xmit_buffer(unsigned int id, dma_addr_t buffer,
		      unsigned int length);
int mcbsp_recv_buffer(unsigned int id, dma_addr_t buffer,
		      unsigned int length);

void mcbsp_tx_dma_callback(int lch, u16 ch_status, void *data);
void mcbsp_rx_dma_callback(int lch, u16 ch_status, void *data);

/* SPI specific API */
void mcbsp_set_spi_mode(unsigned int id,
			const struct mcbsp_spi_cfg *spi_cfg);

/* Law level raw access methods */

#ifdef  MCBSP1_BASE_ADDR 
#define MCBSP_REG_BASE(num) ((unsigned int) (MCBSP0_BASE_ADDR + \
					     ((MCBSP1_BASE_ADDR - MCBSP0_BASE_ADDR) * num)))
#else
#define MCBSP_REG_BASE(num) MCBSP0_BASE_ADDR
#endif

#define mcbsp_setbit_reg(reg, num, val) \
        *((volatile unsigned int *) (MCBSP_REG_BASE(num) + (reg))) |= (unsigned int) (val)
	    
#define mcbsp_clearbit_reg(reg, num, val) \
        *((volatile unsigned int *) (MCBSP_REG_BASE(num) + (reg))) &= ~((unsigned int) (val))
        
#define mcbsp_set_reg(reg, num, val) \
        *((volatile unsigned int *) (MCBSP_REG_BASE(num) + (reg))) = (unsigned int) (val)
        
#define mcbsp_get_reg(reg, num) \
        *((volatile unsigned int *) (MCBSP_REG_BASE(num) + (reg)))

struct mcbsp_info {
	u32 phys_base;
        u8  dma_rx_sync;
        u8  dma_tx_sync;
        u32 dma_rx_data;
        u32 dma_tx_data;
        u16 rx_irq;
        u16 tx_irq;
};

#endif /* __ASM_ARCH_MCBSP_H */
