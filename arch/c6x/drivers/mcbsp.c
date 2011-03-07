/*
 * arch/c6x/drivers/mcbsp.c
 *
 * Copyright (C) 2004 Nokia Corporation
 * Author: Samuel Ortiz <samuel.ortiz@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Multichannel mode not supported.
 *
 * 2005-10-01   Rishi Bhattacharya / Sharath Kumar - Modified to support TI
 *		Davinci DM644x processor
 * 2009-03-30   Aurelien Jacquiot / Nicolas Videau - Modified to support all TI
 *              TMS320C6x based DSP
 *              Copyright (C) 2006, 2009, 2010 Texas Instruments Incorporated
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>

#include <asm/edma.h>
#include <asm/mcbsp.h>

#ifdef CONFIG_MCBSP_DEBUG
#define DBG(x...)       printk(KERN_INFO x)
#else
#define DBG(x...)       do { } while (0)
#endif

struct mcbsp *mcbsp_ptr[MAX_MCBSP_COUNT];

unsigned short MCBSP_READ(int base, int reg)
{
	unsigned long data, temp, *p = (unsigned long *)(base + reg);

	temp = (unsigned long)p;

	if (temp & 0x2) {
		/*non word offset */
		temp &= 0xfffffffc;
		p = (unsigned long *)temp;
		data = *p;
		return (unsigned short)(data >> 16);
	} else {
		/*word offset */
		return ((unsigned short)*p);
	}
}

void MCBSP_WRITE(int base, int reg, unsigned short val)
{
	unsigned long data, temp, *p = (unsigned long *)(base + reg);

	temp = (unsigned long)p;

	if (temp & 0x2) {
		/*non word offset */
		temp &= 0xfffffffc;
		p = (unsigned long *)temp;
		data = *p;
		data &= 0x0000ffff;
		data |= ((unsigned long)val << 16);
		*p = data;
	} else {
		/*word offset */
		data = *p;
		data &= 0xffff0000;
		data |= val;
		*p = data;
	}
}

void mcbsp_dump_reg(u8 id)
{
	DBG("**** MCBSP%d regs ****\n", mcbsp_ptr[id]->id - 1);

	DBG("SPCR2: 0x%04x\n",  MCBSP_READ(mcbsp_ptr[id]->io_base, SPCR2));
	DBG("SPCR1: 0x%04x\n",	MCBSP_READ(mcbsp_ptr[id]->io_base, SPCR1));
	DBG("RCR2:  0x%04x\n",	MCBSP_READ(mcbsp_ptr[id]->io_base, RCR2));
	DBG("RCR1:  0x%04x\n",	MCBSP_READ(mcbsp_ptr[id]->io_base, RCR1));
	DBG("XCR2:  0x%04x\n",	MCBSP_READ(mcbsp_ptr[id]->io_base, XCR2));
	DBG("XCR1:  0x%04x\n",	MCBSP_READ(mcbsp_ptr[id]->io_base, XCR1));
	DBG("SRGR2: 0x%04x\n",	MCBSP_READ(mcbsp_ptr[id]->io_base, SRGR2));
	DBG("SRGR1: 0x%04x\n",	MCBSP_READ(mcbsp_ptr[id]->io_base, SRGR1));
	DBG("PCR0:  0x%04x\n",	MCBSP_READ(mcbsp_ptr[id]->io_base, PCR0));
	DBG("***********************\n");

}

static void mcbsp_tx_dma_callback(unsigned lch, u16 ch_status, void *data)
{
	struct mcbsp *mcbsp_dma_tx = (struct mcbsp *)(data);

	DBG("TX DMA callback : 0x%x\n",
	    MCBSP_READ(mcbsp_dma_tx->io_base, SPCR2));

	/* We can free the channels */
	DBG("mcbsp_dma_tx->dma_tx_lch = %d\n",
	       mcbsp_dma_tx->dma_tx_lch);
	edma_stop(mcbsp_dma_tx->dma_tx_lch);
	edma_free_channel(mcbsp_dma_tx->dma_tx_lch);
	mcbsp_dma_tx->dma_tx_lch = -1;
	complete(&mcbsp_dma_tx->tx_dma_completion);
}

static void mcbsp_rx_dma_callback(unsigned lch, u16 ch_status, void *data)
{
	struct mcbsp *mcbsp_dma_rx = (struct mcbsp *)(data);

	DBG("RX DMA callback : 0x%x\n",
	    MCBSP_READ(mcbsp_dma_rx->io_base, SPCR2));

	/* We can free the channels */
	edma_free_channel(mcbsp_dma_rx->dma_rx_lch);
	mcbsp_dma_rx->dma_rx_lch = -1;

	complete(&mcbsp_dma_rx->rx_dma_completion);
}

/*
 * mcbsp_config simply write a config to the
 * appropriate McBSP.
 * You either call this function or set the McBSP registers
 * by yourself before calling mcbsp_start().
 */
void mcbsp_config(unsigned int id,
		  const struct mcbsp_reg_cfg *config)
{
	u32 io_base = (u32) mcbsp_ptr[id]->io_base;

	DBG("McBSP: McBSP%d  io_base: 0x%08x\n", id, io_base);

	MCBSP_WRITE(io_base, PCR0, config->pcr0);

	MCBSP_WRITE(io_base, RCR2, config->rcr2);
	MCBSP_WRITE(io_base, RCR1, config->rcr1);

	MCBSP_WRITE(io_base, XCR2, config->xcr2);
	MCBSP_WRITE(io_base, XCR1, config->xcr1);

	MCBSP_WRITE(io_base, SRGR2, config->srgr2);
	MCBSP_WRITE(io_base, SRGR1, config->srgr1);

	MCBSP_WRITE(io_base, SPCR2, config->spcr2);
	MCBSP_WRITE(io_base, SPCR1, config->spcr1);

	return;
}

static int mcbsp_check(unsigned int id)
{
	if (id > MAX_MCBSP_COUNT - 1) {
		DBG("McBSP: McBSP%d doesn't exist\n", id + 1);
		return -1;
	}
	return 0;
}

int mcbsp_request(unsigned int id)
{
	if (mcbsp_check(id) < 0)
		return -EINVAL;

	spin_lock(&mcbsp_ptr[id]->lock);
	if (!mcbsp_ptr[id]->free) {
		DBG("McBSP: McBSP%d is currently in use\n", id + 1);
		spin_unlock(&mcbsp_ptr[id]->lock);
		return -1;
	}

	mcbsp_ptr[id]->free = 0;
	spin_unlock(&mcbsp_ptr[id]->lock);

	return 0;

}

void mcbsp_free(unsigned int id)
{
	if (mcbsp_check(id) < 0)
		return;

	spin_lock(&mcbsp_ptr[id]->lock);
	if (mcbsp_ptr[id]->free) {
		DBG("McBSP: McBSP%d was not reserved\n", id + 1);
		spin_unlock(&mcbsp_ptr[id]->lock);
		return;
	}

	mcbsp_ptr[id]->free = 1;
	spin_unlock(&mcbsp_ptr[id]->lock);

	return;
}

/*
 * Here we start the McBSP, by enabling the sample
 * generator, both transmitter and receivers,
 * and the frame sync.
 */
void mcbsp_start(unsigned int id)
{
	u32 io_base;
	u16 w;
	unsigned long flags;
	
	if (mcbsp_check(id) < 0)
		return;

	io_base = (u32) mcbsp_ptr[id]->io_base;

	mcbsp_ptr[id]->rx_word_length =
	    ((MCBSP_READ(io_base, RCR1) >> 5) & 0x7);
	mcbsp_ptr[id]->tx_word_length =
	    ((MCBSP_READ(io_base, XCR1) >> 5) & 0x7);

	local_irq_save(flags);

	/* Start the sample generator */
	w = MCBSP_READ(io_base, SPCR2);
	MCBSP_WRITE(io_base, SPCR2, w | (1 << 6));
		
        /* Enable transmitter and receiver */
	w = MCBSP_READ(io_base, SPCR2);
	MCBSP_WRITE(io_base, SPCR2, w | 1);

	w = MCBSP_READ(io_base, SPCR1);
	MCBSP_WRITE(io_base, SPCR1, w | 1);

	/* Start frame sync */
	w = MCBSP_READ(io_base, SPCR2);
	MCBSP_WRITE(io_base, SPCR2, w | (1 << 7));

	local_irq_restore(flags);

	return;
}

void mcbsp_stop(unsigned int id)
{
	u32 io_base;
	u16 w;
	unsigned long flags;

	if (mcbsp_check(id) < 0)
		return;

	io_base = (u32) mcbsp_ptr[id]->io_base;

	local_irq_save(flags);

	/* Reset transmitter */
	w = MCBSP_READ(io_base, SPCR2);
	MCBSP_WRITE(io_base, SPCR2, w & ~(1));

	/* Reset receiver */
	w = MCBSP_READ(io_base, SPCR1);
	MCBSP_WRITE(io_base, SPCR1, w & ~(1));

	/* Reset the sample rate generator */
	w = MCBSP_READ(io_base, SPCR2);
	MCBSP_WRITE(io_base, SPCR2, w & ~(1 << 6));

	/* Reset the frame sync generator */
	w = MCBSP_READ(io_base, SPCR2);
	MCBSP_WRITE(io_base, SPCR2, w & ~(1 << 7));

	local_irq_restore(flags);

	return;
}

void mcbsp_start_raw(unsigned int id)
{
	u32 io_base;
	u16 w;
	unsigned long flags;

	if (mcbsp_check(id) < 0)
		return;

	io_base = (u32) mcbsp_ptr[id]->io_base;

	mcbsp_ptr[id]->rx_word_length =
	    ((MCBSP_READ(io_base, RCR1) >> 5) & 0x7);
	mcbsp_ptr[id]->tx_word_length =
	    ((MCBSP_READ(io_base, XCR1) >> 5) & 0x7);

	local_irq_save(flags);

	/* Start the sample generator */
	w = MCBSP_READ(io_base, SPCR2);
	MCBSP_WRITE(io_base, SPCR2, w | (1 << 6));

	/* Disable transmitter and receiver */
	w = MCBSP_READ(io_base, SPCR2);
	MCBSP_WRITE(io_base, SPCR2, w & ~1);

	w = MCBSP_READ(io_base, SPCR1);
	MCBSP_WRITE(io_base, SPCR1, w & ~1);

	/* Start frame sync */
	w = MCBSP_READ(io_base, SPCR2);
	MCBSP_WRITE(io_base, SPCR2, w | (1 << 7));

	local_irq_restore(flags);

	return;
}

void mcbsp_start_tx(unsigned int id)
{
	u32 io_base;
	u16 w;
	unsigned long flags;

	if (mcbsp_check(id) < 0)
		return;

	io_base = (u32) mcbsp_ptr[id]->io_base;

	local_irq_save(flags);

	/* Enable transmitter */
	w = MCBSP_READ(io_base, SPCR2);
	MCBSP_WRITE(io_base, SPCR2, w | 1);

	local_irq_restore(flags);

	return;
}

void mcbsp_stop_tx(unsigned int id)
{
	u32 io_base;
	u16 w;
	unsigned long flags;

	if (mcbsp_check(id) < 0)
		return;

	io_base = (u32) mcbsp_ptr[id]->io_base;

	local_irq_save(flags);

	/* Disable transmitter */
	w = MCBSP_READ(io_base, SPCR2);
	MCBSP_WRITE(io_base, SPCR2, w & ~1);

	local_irq_restore(flags);

	return;
}

void mcbsp_start_rx(unsigned int id)
{
	u32 io_base;
	u16 w;
	unsigned long flags;

	if (mcbsp_check(id) < 0)
		return;
	io_base = (u32) mcbsp_ptr[id]->io_base;

	local_irq_save(flags);

	/* Enable receiver */
	w = MCBSP_READ(io_base, SPCR1);
	MCBSP_WRITE(io_base, SPCR1, w | 1);

	local_irq_restore(flags);

	return;
}

void mcbsp_stop_rx(unsigned int id)
{
	u32 io_base;
	u16 w;
	unsigned long flags;

	if (mcbsp_check(id) < 0)
		return;

	io_base = (u32) mcbsp_ptr[id]->io_base;

	local_irq_save(flags);

	/* Disable receiver */
	w = MCBSP_READ(io_base, SPCR1);
	MCBSP_WRITE(io_base, SPCR1, w & ~1);

	local_irq_restore(flags);

	return;
}

/*
 * IRQ based word transmission.
 */
void mcbsp_xmit_word(unsigned int id, u32 word)
{

	u32 io_base;
	mcbsp_word_length word_length = mcbsp_ptr[id]->tx_word_length;

	if (mcbsp_check(id) < 0)
		return;

	io_base = (u32) mcbsp_ptr[id]->io_base;

	DBG(" io_base = 0x%x\n", io_base);
	if (word_length > MCBSP_WORD_16) {
		DBG(" writing DXR2 register \n");
		MCBSP_WRITE(io_base, DXR2, word >> 16);
	}
	DBG(" writing DXR1 register \n");
	MCBSP_WRITE(io_base, DXR1, word & 0xffff);

}

u32 mcbsp_recv_word(unsigned int id)
{

	u32 io_base;
	u16 word_lsb, word_msb = 0;

	mcbsp_word_length word_length = mcbsp_ptr[id]->rx_word_length;

	if (mcbsp_check(id) < 0)
		return -EINVAL;

	io_base = (u32) mcbsp_ptr[id]->io_base;

	if (word_length > MCBSP_WORD_16)
		word_msb = MCBSP_READ(io_base, DRR2);
	word_lsb = MCBSP_READ(io_base, DRR1);

	return (word_lsb | (word_msb << 16));

}

/*
 * Simple DMA based buffer rx/tx routines.
 * Nothing fancy, just a single buffer tx/rx through DMA.
 * The DMA resources are released once the transfer is done.
 * For anything fancier, you should use your own customized DMA
 * routines and callbacks.
 */
int mcbsp_xmit_buffer(unsigned int id, dma_addr_t buffer,
		      unsigned int length)
{
	int dma_tx_ch;

	if (mcbsp_check(id) < 0)
		return -EINVAL;

	dma_tx_ch = edma_alloc_channel(mcbsp_ptr[id]->dma_tx_sync,
				       mcbsp_tx_dma_callback, mcbsp_ptr[id],
				       EVENTQ_3);
	if (dma_tx_ch < 0) {
		DBG("McBSP: Unable to request DMA channel for McBSP%d TX. "
		    "Trying IRQ based TX\n", id + 1);
		return -EAGAIN;
	}
	mcbsp_ptr[id]->dma_tx_lch = dma_tx_ch;

	INIT_COMPLETION(mcbsp_ptr[id]->tx_dma_completion);

	edma_set_transfer_params(dma_tx_ch, 2, length / 2, 1, 0, ASYNC);
	edma_set_dest(dma_tx_ch, mcbsp_ptr[id]->dma_tx_data, 0, 0);

	edma_set_src(dma_tx_ch, buffer, 0, 0);
	edma_set_src_index(dma_tx_ch, 2, 0);
	edma_set_dest_index(dma_tx_ch, 0, 0);

	edma_start(dma_tx_ch);

	wait_for_completion(&(mcbsp_ptr[id]->tx_dma_completion));

	return 0;
}

int mcbsp_recv_buffer(unsigned int id, dma_addr_t buffer,
		      unsigned int length)
{
	int dma_rx_ch;

	if (mcbsp_check(id) < 0)
		return -EINVAL;

	dma_rx_ch = edma_alloc_channel(mcbsp_ptr[id]->dma_rx_sync,
				       mcbsp_rx_dma_callback, mcbsp_ptr[id],
				       EVENTQ_3);
	if (dma_rx_ch < 0) {
		DBG("McBSP: Unable to request DMA channel for McBSP%d RX. "
		    "Trying IRQ based RX\n", id + 1);
		return -EAGAIN;
	}
	mcbsp_ptr[id]->dma_rx_lch = dma_rx_ch;

	DBG("RX DMA on channel %d\n", dma_rx_ch);

	INIT_COMPLETION(mcbsp_ptr[id]->rx_dma_completion);

	edma_set_transfer_params(dma_rx_ch, 2, length / 2, 1, 0, ASYNC);

	edma_set_src(dma_rx_ch, mcbsp_ptr[id]->dma_rx_data, 0, 0);
				   
	edma_set_dest(dma_rx_ch, (unsigned long)virt_to_phys((void *)buffer),
		      0, 0);
	edma_set_src_index(dma_rx_ch, 0, 0);
	edma_set_dest_index(dma_rx_ch, 2, 0);

	edma_start(dma_rx_ch);

	wait_for_completion(&(mcbsp_ptr[id]->rx_dma_completion));
	DBG(" mcbsp_recv_buffer: after wait_for_completion\n");
	return 0;
}

struct clk * mcbsp_get_clock(unsigned int id)
{
	return mcbsp_ptr[id]->clk;
}

/*
 * SPI wrapper.
 * Since SPI setup is much simpler than the generic McBSP one,
 * this wrapper just need an mcbsp_spi_cfg structure as an input.
 * Once this is done, you can call mcbsp_start().
 */
void mcbsp_set_spi_mode(unsigned int id, const struct mcbsp_spi_cfg * spi_cfg)
{
	struct mcbsp_reg_cfg mcbsp_cfg;

	if (mcbsp_check(id) < 0)
		return;

	memset(&mcbsp_cfg, 0, sizeof(struct mcbsp_reg_cfg));

	/* SPI has only one frame */
	mcbsp_cfg.rcr1 |= (RWDLEN1(spi_cfg->word_length) | RFRLEN1(0));
	mcbsp_cfg.xcr1 |= (XWDLEN1(spi_cfg->word_length) | XFRLEN1(0));

        /* Clock stop mode */
	if (spi_cfg->clk_stp_mode == MCBSP_CLK_STP_MODE_NO_DELAY)
		mcbsp_cfg.spcr1 |= (1 << 12);
	else
		mcbsp_cfg.spcr1 |= (3 << 11);

	/* Set clock parities */
	if (spi_cfg->rx_clock_polarity == MCBSP_CLK_RISING)
		mcbsp_cfg.pcr0 |= CLKRP;
	else
		mcbsp_cfg.pcr0 &= ~CLKRP;

	if (spi_cfg->tx_clock_polarity == MCBSP_CLK_RISING)
		mcbsp_cfg.pcr0 &= ~CLKXP;
	else
		mcbsp_cfg.pcr0 |= CLKXP;

	/* Set SCLKME to 0 and CLKSM to 1 */
	mcbsp_cfg.pcr0 &= ~SCLKME;
	mcbsp_cfg.srgr2 |= CLKSM;

	/* Set FSXP */
	if (spi_cfg->fsx_polarity == MCBSP_FS_ACTIVE_HIGH)
		mcbsp_cfg.pcr0 &= ~FSXP;
	else
		mcbsp_cfg.pcr0 |= FSXP;

	/* Set format mode, LSB-first is only supported for 32 bits word length */
	if ((spi_cfg->format_mode == MCBSP_LSB_FIRST) && (spi_cfg->word_length == 5)) {
	        mcbsp_cfg.xcr1 |= XWDREVRS;
		mcbsp_cfg.xcr2 |= XCOMPAND(1);
	} else {
	        mcbsp_cfg.xcr1 &= ~XWDREVRS;
		mcbsp_cfg.xcr2 &= ~XCOMPAND(3);
	}

	if (spi_cfg->spi_mode == MCBSP_SPI_MASTER) {
		mcbsp_cfg.pcr0 |= CLKXM;
		mcbsp_cfg.srgr1 |= CLKGDV(spi_cfg->clk_div - 1);
		mcbsp_cfg.pcr0 |= FSXM;
		mcbsp_cfg.srgr2 &= ~FSGM;
		mcbsp_cfg.xcr2 |= XDATDLY(1);
		mcbsp_cfg.rcr2 |= RDATDLY(1);
	}
	else {
		mcbsp_cfg.pcr0 &= ~CLKXM;
		mcbsp_cfg.srgr1 |= CLKGDV(1);
		mcbsp_cfg.pcr0 &= ~FSXM;
		mcbsp_cfg.xcr2 &= ~XDATDLY(3);
		mcbsp_cfg.rcr2 &= ~RDATDLY(3);
	}

	mcbsp_cfg.xcr2 &= ~XPHASE;
	mcbsp_cfg.rcr2 &= ~RPHASE;

	mcbsp_config(id, &mcbsp_cfg);
}

static int __devinit mcbsp_probe(struct platform_device *pdev)
{
	struct mcbsp_info *pdata = (struct mcbsp_info *) pdev->dev.platform_data;
	struct mcbsp      *mcbsp;
	int                id    = pdev->id - 1;
	int                ret   = 0;

	if (!pdata) {
		dev_err(&pdev->dev, "McBSP device initialized without"
			"platform data\n");
		ret = -EINVAL;
		goto exit;
	}

	dev_dbg(&pdev->dev, "Initializing McBSP (%d).\n", pdev->id);

	if (id >= MAX_MCBSP_COUNT) {
		dev_err(&pdev->dev, "Invalid McBSP device id (%d)\n", id);
		ret = -EINVAL;
		goto exit;
	}

	mcbsp = kzalloc(sizeof(struct mcbsp), GFP_KERNEL);
	if (!mcbsp) {
		ret = -ENOMEM;
		goto exit;
	}

	spin_lock_init(&mcbsp->lock);
	mcbsp->id = id + 1;
	mcbsp->free = 1;
	mcbsp->dma_tx_lch = -1;
	mcbsp->dma_rx_lch = -1;

	mcbsp->io_base = ioremap(pdata->phys_base, 0x1000);
	if (!mcbsp->io_base) {
		ret = -ENOMEM;
		goto err_ioremap;
	}

	mcbsp->tx_irq = pdata->tx_irq;
	mcbsp->rx_irq = pdata->rx_irq;
	mcbsp->dma_rx_sync = pdata->dma_rx_sync;
	mcbsp->dma_tx_sync = pdata->dma_tx_sync;
	mcbsp->dma_rx_data = pdata->dma_rx_data;
	mcbsp->dma_tx_data = pdata->dma_tx_data;

	init_completion(&(mcbsp->rx_dma_completion));
	init_completion(&(mcbsp->tx_dma_completion));

	mcbsp->clk = clk_get(&pdev->dev, "mcbsp");
	if (IS_ERR(mcbsp->clk)) {
		ret = PTR_ERR(mcbsp->clk);
		dev_err(&pdev->dev, "unable to get clk: %d\n", ret);
		goto err_clk;
	}

	mcbsp->pdata  = pdata;
	mcbsp->dev    = &pdev->dev;
	mcbsp_ptr[id] = mcbsp;

	platform_set_drvdata(pdev, mcbsp);

	return 0;

err_clk:
	iounmap(mcbsp->io_base);
err_ioremap:
	kfree(mcbsp);
exit:
	return ret;
}

static int __devexit mcbsp_remove(struct platform_device *pdev)
{
	struct mcbsp *mcbsp = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	if (mcbsp) {
		clk_disable(mcbsp->clk);
		clk_put(mcbsp->clk);

		iounmap(mcbsp->io_base);

		mcbsp->clk = NULL;
		mcbsp->free = 0;
		mcbsp->dev = NULL;
	}

	return 0;
}

static struct platform_driver mcbsp_driver = {
	.probe		= mcbsp_probe,
	.remove		= __devexit_p(mcbsp_remove),
	.driver		= {
		.name	= "mcbsp",
	},
};

int __init mcbsp_init(void)
{
	/* Register the McBSP driver */
	return platform_driver_register(&mcbsp_driver);
}

module_init(mcbsp_init);

EXPORT_SYMBOL(mcbsp_config);
EXPORT_SYMBOL(mcbsp_request);
EXPORT_SYMBOL(mcbsp_free);
EXPORT_SYMBOL(mcbsp_start);
EXPORT_SYMBOL(mcbsp_start_raw);
EXPORT_SYMBOL(mcbsp_start_tx);
EXPORT_SYMBOL(mcbsp_start_rx);
EXPORT_SYMBOL(mcbsp_stop);
EXPORT_SYMBOL(mcbsp_stop_tx);
EXPORT_SYMBOL(mcbsp_stop_rx);
EXPORT_SYMBOL(mcbsp_xmit_word);
EXPORT_SYMBOL(mcbsp_recv_word);
EXPORT_SYMBOL(mcbsp_xmit_buffer);
EXPORT_SYMBOL(mcbsp_recv_buffer);
EXPORT_SYMBOL(mcbsp_get_clock);
EXPORT_SYMBOL(mcbsp_tx_dma_callback);
EXPORT_SYMBOL(mcbsp_rx_dma_callback);
EXPORT_SYMBOL(mcbsp_set_spi_mode);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("McBSP driver");
MODULE_LICENSE("GPL");
