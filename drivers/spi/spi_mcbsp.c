/*
 *  drivers/spi/spi_mcbsp.c
 *
 *  SPI interface driver using emulation on McBSP interface
 *
 *  Copyright (C) 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>

#include <asm/edma.h>
#include <asm/mcbsp.h>

#ifdef CONFIG_EVM6488_PCF857X
#include <asm/evm6488-i2c.h>
#include <asm/gpio.h>
#endif

#define START_STATE          ((void*)0)
#define RUNNING_STATE        ((void*)1)
#define DONE_STATE           ((void*)2)
#define ERROR_STATE          ((void*)-1)
#define QUEUE_RUNNING        0
#define QUEUE_STOPPED        1

static int zero = 0;

struct spi_mcbsp {

        /* McBSP device driver stuff */ 
        int    mcbsp_id;
        struct mcbsp *mcbsp;
        struct mcbsp_spi_cfg mcbsp_cfg;
        u16    chip_select;
        u16    bits_per_word;
        u32    speed_hz;
	struct clk *clk;

	/* SPI framework hookup */
        struct device *dev;
	struct spi_master *master;
        u32    first_setup_done;
	u32    cfg_modified;

        /* SPI framework message queue */
	struct workqueue_struct *workqueue;
	struct work_struct pump_messages;
	spinlock_t lock;
	struct list_head queue;
	int busy;
	int run;

        /* SPI framework message transfer pump */
	struct tasklet_struct pump_transfers;
	struct spi_message  *cur_msg;
	struct spi_transfer *cur_transfer;

	/* Rd / Wr buffers pointers */
	size_t tx_len;
	void  *tx;
	void  *tx_end;
	size_t rx_len;
	void  *rx;
	void  *rx_end;

        /* EDMA setup */
	dma_addr_t rx_dma;
	dma_addr_t tx_dma;
        struct edmacc_param dma_tx_params;
        struct edmacc_param dma_rx_params;
};

static inline struct spi_mcbsp* get_spi_mcbsp(struct spi_device *spi) 
{
        return ((struct spi_mcbsp *) spi_master_get_devdata(spi->master));
}

static inline int get_mcbsp_id(struct spi_device *spi)
{
        struct spi_mcbsp *spi_mcbsp = get_spi_mcbsp(spi);
        return spi_mcbsp->mcbsp_id;
}

static inline void set_mcbsp_id(struct spi_device *spi, int id)
{
        struct spi_mcbsp *spi_mcbsp = get_spi_mcbsp(spi);
        spi_mcbsp->mcbsp_id = id;
}

/* Convert SPI frequency into McBSP clock divider */
static inline unsigned int hz_to_spi_clk_div(unsigned int hz, struct clk *clk)
{
        return (clk_get_rate(clk) / hz);
}

static inline void change_spi_mode(struct spi_mcbsp *spi_mcbsp, struct spi_device *spi)
{
        spi_mcbsp->mcbsp_cfg.word_length = spi->bits_per_word == 32 ? 5 :(spi->bits_per_word - 8) >> 2;
	spi_mcbsp->mcbsp_cfg.spi_mode    = MCBSP_SPI_MASTER;

	/* Translate common spi framework into our mcbsp one */
	if (spi->mode & SPI_CPOL) {
	         spi_mcbsp->mcbsp_cfg.rx_clock_polarity = MCBSP_CLK_FALLING;
	         spi_mcbsp->mcbsp_cfg.tx_clock_polarity = MCBSP_CLK_FALLING;
	} else {
	         spi_mcbsp->mcbsp_cfg.rx_clock_polarity = MCBSP_CLK_RISING;
	         spi_mcbsp->mcbsp_cfg.tx_clock_polarity = MCBSP_CLK_RISING;
	}

	if (spi->mode & SPI_CPHA)
	         spi_mcbsp->mcbsp_cfg.clk_stp_mode = MCBSP_CLK_STP_MODE_NO_DELAY;    /* CPHA=1 */
	else
	         spi_mcbsp->mcbsp_cfg.clk_stp_mode = MCBSP_CLK_STP_MODE_DELAY;       /* CPHA=0 */

	if (spi->mode & SPI_CS_HIGH)
	         spi_mcbsp->mcbsp_cfg.fsx_polarity = MCBSP_FS_ACTIVE_HIGH;
	else
	         spi_mcbsp->mcbsp_cfg.fsx_polarity = MCBSP_FS_ACTIVE_LOW;

	if (spi->mode & SPI_LSB_FIRST)
	         spi_mcbsp->mcbsp_cfg.format_mode = MCBSP_LSB_FIRST;
	else
	         spi_mcbsp->mcbsp_cfg.format_mode = MCBSP_MSB_FIRST;	     

        /* Compute clock divider */ 
	spi_mcbsp->mcbsp_cfg.clk_div =
		hz_to_spi_clk_div(spi->max_speed_hz, spi_mcbsp->clk) & 0xff;

	dev_dbg(spi_mcbsp->dev, "%s: freq = %d, clk_div = 0x%x\n", 
		__FUNCTION__, spi->max_speed_hz, spi_mcbsp->mcbsp_cfg.clk_div);
}

/*
 * Caller already set message->status;
 */
static void spi_mcbsp_giveback(struct spi_message *message, struct spi_mcbsp *spi_mcbsp)
{
        u16 w;

	/* shutdown CS */
	w = MCBSP_READ((int) spi_mcbsp->mcbsp->io_base, PCR0);
	MCBSP_WRITE((int)spi_mcbsp->mcbsp->io_base, PCR0, (w & ~FSXM));

	/* Stop SPI */
	mcbsp_stop(spi_mcbsp->mcbsp_id);

	dev_dbg(spi_mcbsp->dev, "%s called\n",__FUNCTION__);
	
	message->state = NULL;
	if (message->complete)
		message->complete(message->context);

	spi_mcbsp->cur_msg      = NULL;
	spi_mcbsp->cur_transfer = NULL;
	queue_work(spi_mcbsp->workqueue, &spi_mcbsp->pump_messages);
}

/*
 * Test if there is more transfer to be done
 */
static void *spi_mcbsp_next_transfer(struct spi_mcbsp *spi_mcbsp)
{
        struct spi_message  *msg   = spi_mcbsp->cur_msg;
	struct spi_transfer *trans = spi_mcbsp->cur_transfer;

	/* Move to next transfer */
	if (trans->transfer_list.next != &msg->transfers) {
	        spi_mcbsp->cur_transfer =
		        list_entry(trans->transfer_list.next,
				   struct spi_transfer, transfer_list);
		return RUNNING_STATE;
	} 
	return DONE_STATE;
}

/*
 * EDMA management
 */
static void spi_mcbsp_edma_tx_callback(unsigned int lch, u16 ch_status, void *data)
{
        struct spi_mcbsp   *spi_mcbsp = (struct spi_mcbsp *) (data);
	struct spi_message *msg       = spi_mcbsp->cur_msg;

	/* Wait transmit to be physically finished */
	while ((MCBSP_READ((int) spi_mcbsp->mcbsp->io_base, SPCR2) & XEMPTY));

	if ((spi_mcbsp->tx_len) && (!spi_mcbsp->rx_len)) {

	        /* Real TX case */
	        msg->actual_length += spi_mcbsp->tx_len;

	        /* Move to next transfer */
	        msg->state = spi_mcbsp_next_transfer(spi_mcbsp);

		/* Schedule transfer tasklet */
		tasklet_schedule(&spi_mcbsp->pump_transfers);
	}
}

static void spi_mcbsp_edma_rx_callback(unsigned int lch, u16 ch_status, void *data)
{
        struct spi_mcbsp   *spi_mcbsp = (struct spi_mcbsp *) (data);
	struct spi_message *msg       = spi_mcbsp->cur_msg;

	/* Wait receive to be physically finished */
	while ((MCBSP_READ((int)spi_mcbsp->mcbsp->io_base, SPCR1) & RRDY));

	if (spi_mcbsp->rx_len) {

	        /* No coherency is assumed between EDMA and L2 cache */
	        L2_cache_block_invalidate((u32) spi_mcbsp->rx_dma,
					  (u32) spi_mcbsp->rx_dma + spi_mcbsp->rx_len);

		msg->actual_length += spi_mcbsp->rx_len;

		/* Move to next transfer */
		msg->state = spi_mcbsp_next_transfer(spi_mcbsp);

		/* Schedule transfer tasklet */
		tasklet_schedule(&spi_mcbsp->pump_transfers);
	}
}

static void spi_mcbsp_pump_transfers(unsigned long data)
{
    	struct spi_mcbsp    *spi_mcbsp = (struct spi_mcbsp *) data;
	struct spi_message  *message   = NULL;
	struct spi_transfer *transfer  = NULL;
	struct spi_transfer *previous  = NULL;
	struct mcbsp        *mcbsp;
	u32 mcbsp_id;
	u32 len;
	u32 width;

	/* Get current state information */
	message  = spi_mcbsp->cur_msg;
	transfer = spi_mcbsp->cur_transfer;

	/*
	 * If msg is error or done, report it back using complete() callback
	 */

	/* Handle for abort */
	if (message->state == ERROR_STATE) {
		message->status = -EIO;
		spi_mcbsp_giveback(message, spi_mcbsp);
		return;
	}

	/* Handle end of message */
	if (message->state == DONE_STATE) {
		message->status = 0;
		spi_mcbsp_giveback(message, spi_mcbsp);
		return;
	}

	/* Delay if requested at the end of transfer */
	if (message->state == RUNNING_STATE) {
		previous = list_entry(transfer->transfer_list.prev,
				      struct spi_transfer, transfer_list);
		if (previous->delay_usecs)
			udelay(previous->delay_usecs);
	}

	/* Setup the McBSP transfer */
	message->state    = RUNNING_STATE;
	mcbsp_id          = get_mcbsp_id(message->spi); /* Get the McBSP number */
	mcbsp             = spi_mcbsp->mcbsp;
	spi_mcbsp->tx     = (void *) transfer->tx_buf;
	spi_mcbsp->tx_end = spi_mcbsp->tx + transfer->len;
	spi_mcbsp->rx     = (void *) transfer->rx_buf;
	spi_mcbsp->rx_end = spi_mcbsp->rx + transfer->len;
	spi_mcbsp->rx_dma = (dma_addr_t) transfer->rx_buf;
	spi_mcbsp->tx_dma = (dma_addr_t) transfer->tx_buf;
	spi_mcbsp->tx_len = 0;
	spi_mcbsp->rx_len = 0;

	if (!(spi_mcbsp->tx) && !(spi_mcbsp->rx)) {
	        message->status = -EINVAL;
		spi_mcbsp_giveback(message, spi_mcbsp);
		return;
	}
	    
	dev_dbg(spi_mcbsp->dev,
		"now pumping a transfer to McBSP%d: len is %d\n",
		mcbsp_id, transfer->len);

	/* Check if the chip select has been changed for this message */
	dev_dbg(spi_mcbsp->dev, "current CS = %d, message->spi CS %d\n",
		spi_mcbsp->chip_select,
		message->spi->chip_select);

	if ((spi_mcbsp->cfg_modified == 1) || (message->spi->chip_select != spi_mcbsp->chip_select)) {
	              
           	    change_spi_mode(spi_mcbsp, message->spi);

		    dev_dbg(spi_mcbsp->dev, "CS changed from %d to %d\n",
			    spi_mcbsp->chip_select,
			    message->spi->chip_select);

		    spi_mcbsp->chip_select   = message->spi->chip_select;
		    spi_mcbsp->bits_per_word = message->spi->bits_per_word;
		    spi_mcbsp->speed_hz      = message->spi->max_speed_hz;
		    spi_mcbsp->cfg_modified  = 0;

		    dev_dbg(spi_mcbsp->dev, "bits_per_word = %d, word_length = %d\n", 
			    spi_mcbsp->bits_per_word,
			    spi_mcbsp->mcbsp_cfg.word_length);
	}
	 
	if (transfer->speed_hz != spi_mcbsp->speed_hz) {
	            spi_mcbsp->mcbsp_cfg.clk_div =
			    hz_to_spi_clk_div(transfer->speed_hz, spi_mcbsp->clk) & 0xff;
		    spi_mcbsp->speed_hz = transfer->speed_hz;
		    dev_dbg(spi_mcbsp->dev, "modifying clk_div = %x\n", spi_mcbsp->mcbsp_cfg.clk_div);
	}

	if (transfer->bits_per_word != spi_mcbsp->bits_per_word) {
	            spi_mcbsp->mcbsp_cfg.word_length = transfer->bits_per_word == 32 ? 5 :
			(transfer->bits_per_word - 8) >> 2;
		    spi_mcbsp->bits_per_word = transfer->bits_per_word;
		    dev_dbg(spi_mcbsp->dev, "modifying word_length = %x\n", spi_mcbsp->mcbsp_cfg.word_length);
	}

	mcbsp_stop(mcbsp_id);
	mcbsp_set_spi_mode(mcbsp_id, &spi_mcbsp->mcbsp_cfg);
	mcbsp_start_raw(mcbsp_id);

	width = transfer->bits_per_word >> 3; /* width is in bytes */ 
	len   = transfer->len / width;
	
	/* Write parameters */
	edma_write_slot(mcbsp->dma_tx_lch, &spi_mcbsp->dma_tx_params);
	edma_set_transfer_params(mcbsp->dma_tx_lch, width, len, 1, 0, ASYNC);
	
	if (spi_mcbsp->rx) {

	        spi_mcbsp->rx_len = transfer->len;

	        if (spi_mcbsp->tx == NULL) {
		        /* Fake write */
			edma_set_src(mcbsp->dma_tx_lch, (int)&zero, 0, 0);
			edma_set_src_index(mcbsp->dma_tx_lch, 0, 0); /* do no increment src */
		}

		/* Read parameters */
		edma_write_slot(mcbsp->dma_rx_lch, &spi_mcbsp->dma_rx_params);
		edma_set_transfer_params(mcbsp->dma_rx_lch, width, len, 1, 0, ASYNC);
		edma_set_dest(mcbsp->dma_rx_lch, spi_mcbsp->rx_dma, 0, 0);
		edma_set_dest_index(mcbsp->dma_rx_lch, width, 0);
		
		dev_dbg(spi_mcbsp->dev, "starting EDMA RX\n");
		
		/* Start EDMA (RX) */
		edma_start(mcbsp->dma_rx_lch);
		
		/* Start McBSP RX and wait EDMA ending */
		mcbsp_start_rx(mcbsp_id);
	} 

	if (spi_mcbsp->tx) {

	        spi_mcbsp->tx_len = transfer->len;

	        /* Real write */
	        edma_set_src(mcbsp->dma_tx_lch, spi_mcbsp->tx_dma, 0, 0);
		edma_set_src_index(mcbsp->dma_tx_lch, width, 0);
		
		/* No coherency is assumed between EDMA and L2 cache */
		L2_cache_block_writeback((u32) spi_mcbsp->tx_dma,
					 (u32) spi_mcbsp->tx_dma + spi_mcbsp->tx_len);

	}	
	dev_dbg(spi_mcbsp->dev, "starting EDMA TX\n");
	
	/* Start EDMA (TX) */
	edma_start(mcbsp->dma_tx_lch);
	
	/* Start McBSP TX and wait EDMA ending */
	mcbsp_start_tx(mcbsp_id);
}

/* Pop a msg from queue and kick off real transfer */
static void spi_mcbsp_pump_messages(struct work_struct *work)
{
        struct spi_mcbsp *spi_mcbsp = container_of(work, struct spi_mcbsp, pump_messages);
	unsigned long flags;

	/* Lock queue and check for queue work */
	spin_lock_irqsave(&spi_mcbsp->lock, flags);
	if (list_empty(&spi_mcbsp->queue) || spi_mcbsp->run == QUEUE_STOPPED) {
	        /* Pumper kicked off but no work to do */
	        spi_mcbsp->busy = 0;
		spin_unlock_irqrestore(&spi_mcbsp->lock, flags);
		return;
	}

	/* Make sure we are not already running a message */
	if (spi_mcbsp->cur_msg) {
		spin_unlock_irqrestore(&spi_mcbsp->lock, flags);
		return;
	}

	/* Extract head of queue */
	spi_mcbsp->cur_msg = list_entry(spi_mcbsp->queue.next,
					struct spi_message, queue);
	list_del_init(&spi_mcbsp->cur_msg->queue);

	/* Initial message state */
	spi_mcbsp->cur_msg->state = START_STATE;
	spi_mcbsp->cur_transfer   = list_entry(spi_mcbsp->cur_msg->transfers.next,
					       struct spi_transfer, transfer_list);

	spi_mcbsp->cur_transfer->bits_per_word = spi_mcbsp->cur_msg->spi->bits_per_word;
	spi_mcbsp->cur_transfer->speed_hz      = spi_mcbsp->cur_msg->spi->max_speed_hz;
	
	dev_dbg(spi_mcbsp->dev, 
		"the first transfer len is %d\n",
		spi_mcbsp->cur_transfer->len);

	/* Call platform specific function */
	if (spi_mcbsp->cur_msg->spi->dev.platform_data != NULL) {
	        void (*platform_hook)(int) = spi_mcbsp->cur_msg->spi->dev.platform_data;
		platform_hook((int) spi_mcbsp->cur_transfer->rx_buf); /* 0 for write */
	}

	/* Mark as busy and launch transfers */
	tasklet_schedule(&spi_mcbsp->pump_transfers);

	spi_mcbsp->busy = 1;
	spin_unlock_irqrestore(&spi_mcbsp->lock, flags);
}

static int spi_mcbsp_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct spi_mcbsp *spi_mcbsp = spi_master_get_devdata(spi->master);
	unsigned long flags;

	spin_lock_irqsave(&spi_mcbsp->lock, flags);

	if (spi_mcbsp->run == QUEUE_STOPPED) {
		spin_unlock_irqrestore(&spi_mcbsp->lock, flags);
		return -ESHUTDOWN;
	}

	msg->actual_length = 0;
	msg->status        = -EINPROGRESS;
	msg->state         = START_STATE;

	dev_dbg(&spi->dev, "adding an msg in transfer() \n");
	list_add_tail(&msg->queue, &spi_mcbsp->queue);

	if (spi_mcbsp->run == QUEUE_RUNNING && !spi_mcbsp->busy)
		queue_work(spi_mcbsp->workqueue, &spi_mcbsp->pump_messages);

	spin_unlock_irqrestore(&spi_mcbsp->lock, flags);

	return 0;
}

static int spi_mcbsp_setup(struct spi_device *spi)
{
	unsigned int         mcbsp_id  = get_mcbsp_id(spi);
	struct spi_mcbsp    *spi_mcbsp = get_spi_mcbsp(spi);
	struct mcbsp        *mcbsp     = spi_mcbsp->mcbsp;
        int                  dma_tx_ch, dma_tx_ch_reload;
        int                  dma_rx_ch, dma_rx_ch_reload;
	struct edmacc_param  tmp_params;

	/* Zero (the default) here means 8 bits */
	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if ((spi->bits_per_word < 8) || (spi->bits_per_word > 32))
		return -EINVAL;

	/* get McBSP clock */
	spi_mcbsp->clk = mcbsp_get_clock(mcbsp_id);

	if ((hz_to_spi_clk_div(spi->max_speed_hz, spi_mcbsp->clk) < 0)
	    || (hz_to_spi_clk_div(spi->max_speed_hz, spi_mcbsp->clk) > 255)) {
	        /* 
		 * For requested sampling rate, the input clock to MCBSP cant be derived
		 * down to get the in range clock devider value for 16 bits sample
		 */
                dev_dbg(&spi->dev, "Invalid frequency %d requested\n",
                        (int) spi->max_speed_hz);
                return -EPERM;
	}

	spi_mcbsp->cfg_modified  = 1;
		    
	dev_dbg(&spi->dev, "setup called\n");

	if (spi_mcbsp->first_setup_done)
	        return 0;

	/* Request the corresponding McBSP */
	if (mcbsp_request(mcbsp_id))
	        return -EBUSY;

#if (defined(CONFIG_MCBSP_UART) && defined(CONFIG_EVM6488_PCF857X))
        if (mcbsp_id == 0) {
	        /* In case of the McBSP0, we need to set the CS on SPI instead of COM0 */
	        struct i2c_client *client;
		struct gpio_chip  *gpio;

		client = evm6488_i2c_get_client(NULL, PCF8515_I2C_ID);
		if (!client) {
		        dev_warn(&spi->dev, "Unable to get PCF8515 I2C device\n");
		        goto skip_gpio_cs;
		}

		gpio = (struct gpio_chip *) i2c_get_clientdata(client);
		if (!gpio) {
		        dev_warn(&spi->dev, "Unable to get PCF8515 GPIO device\n");
		        goto skip_gpio_cs;
		}

	        /* Use GPIO Expander to enable SPI */
		gpio->set(gpio, 0, 0); /* COM0 PWR D8 */
		gpio->set(gpio, 1, 0); /* COM0 DR0/FSR0 D9 */

		dev_dbg(&spi->dev, "McBSP0 set to SPI\n");
	}
#endif
	/* Allocate the TX EDMA channel */
	dma_tx_ch = edma_alloc_channel(mcbsp->dma_tx_sync,
				       spi_mcbsp_edma_tx_callback,
				       spi_mcbsp,
				       EVENTQ_3);

	if (dma_tx_ch < 0) {
	        dev_dbg(&spi->dev, "Unable to request DMA channel for McBSP%d Tx\n",
			mcbsp_id);
		return -EAGAIN;
	}

	/* Request the dummy reload slot to avoid missed events */
	dma_tx_ch_reload = edma_alloc_slot(EDMA_CTLR(dma_tx_ch), EDMA_SLOT_ANY);
	if (dma_tx_ch_reload < 0) {
	        dev_dbg(&spi->dev, "Unable to request EDMA slot for McBSP%d Tx\n", mcbsp_id);
		return -EAGAIN;
	}
	
	mcbsp->dma_tx_lch        = (short) dma_tx_ch;
	mcbsp->dma_tx_lch_reload = (short) dma_tx_ch_reload;

	/* EDMA TX dest is McBSP0 */
	edma_set_dest(dma_tx_ch, mcbsp->dma_tx_data, 0, 0);
	edma_set_dest_index(dma_tx_ch, 0, 0);
		
	edma_link(dma_tx_ch, dma_tx_ch_reload);

	/* Save TX channel parameter set */
	edma_read_slot(dma_tx_ch, &spi_mcbsp->dma_tx_params);

	/* Add transfer completion interrupt */
	spi_mcbsp->dma_tx_params.opt |= TCINTEN | EDMA_TCC(dma_tx_ch);
	edma_write_slot(dma_tx_ch, &spi_mcbsp->dma_tx_params);

	edma_set_dest(dma_tx_ch_reload, 0, 0, 0);
	edma_set_dest_index(dma_tx_ch_reload, 0, 0);
	edma_set_src_index(dma_tx_ch_reload, 0, 0);
	edma_set_transfer_params(dma_tx_ch_reload,
				 0, /* ACNT */
				 0, /* BCNT */
				 1, /* CCNT */    
				 0, /* BCNTRLD */
				 ASYNC);

	/* Set dummy slot properties */
	edma_read_slot(dma_tx_ch_reload, &tmp_params);
	tmp_params.opt |= TCINTEN| STATIC | TCCMODE | EDMA_TCC(dma_tx_ch);
	edma_write_slot(dma_tx_ch_reload, &tmp_params);
	
	/* Allocate the RX EDMA channel */
	dma_rx_ch = edma_alloc_channel(mcbsp->dma_rx_sync,
				       spi_mcbsp_edma_rx_callback,
				       spi_mcbsp,
				       EVENTQ_3);
	if (dma_rx_ch < 0) {
		dev_dbg(&spi->dev, "Unable to request DMA channel for McBSP%d RX\n",
			mcbsp_id);
		return -EAGAIN;
	}

	/* Request the dummy reload slot to avoid missed events */
	dma_rx_ch_reload = edma_alloc_slot(EDMA_CTLR(dma_rx_ch), EDMA_SLOT_ANY);
	if (dma_rx_ch_reload < 0) {
	        dev_dbg(&spi->dev, "Unable to request EDMA slot for McBSP%d Rx\n", mcbsp_id);
		return -EAGAIN;
	}
	
	mcbsp->dma_rx_lch        = (short) dma_rx_ch;
	mcbsp->dma_rx_lch_reload = (short) dma_rx_ch_reload;

	/* EDMA RX source is McBSP0 */
	edma_set_src(dma_rx_ch, mcbsp->dma_rx_data, 0, 0);
	edma_set_src_index(dma_rx_ch, 0, 0);
		
	edma_link(dma_rx_ch, dma_rx_ch_reload);

	/* Save RX channel parameter set */
	edma_read_slot(dma_rx_ch, &spi_mcbsp->dma_rx_params);

	/* Add transfer completion interrupt */
	spi_mcbsp->dma_rx_params.opt |= TCINTEN | EDMA_TCC(dma_rx_ch);
	edma_write_slot(dma_rx_ch, &spi_mcbsp->dma_rx_params);

	edma_set_dest(dma_rx_ch_reload, 0, 0, 0);
	edma_set_dest_index(dma_rx_ch_reload, 0, 0);
	edma_set_src_index(dma_rx_ch_reload, 0, 0);
	edma_set_transfer_params(dma_rx_ch_reload,
				 0, /* ACNT */
				 0, /* BCNT */
				 1, /* CCNT */    
				 0, /* BCNTRLD */
				 ASYNC);

	/* Set dummy slot properties */
	edma_read_slot(dma_rx_ch_reload, &tmp_params);
	tmp_params.opt |= TCINTEN| STATIC | TCCMODE | EDMA_TCC(dma_rx_ch);
	edma_write_slot(dma_rx_ch_reload, &tmp_params);
	
	/* Just in case... */
	edma_stop(dma_tx_ch); 
	edma_stop(dma_rx_ch); 

	spi_mcbsp->first_setup_done = 1;

	return 0;
}

/*
 * SPI Framework
 */
static inline int init_queue(struct spi_mcbsp *spi_mcbsp)
{
	INIT_LIST_HEAD(&spi_mcbsp->queue);
	spin_lock_init(&spi_mcbsp->lock);

	spi_mcbsp->run  = QUEUE_STOPPED;
	spi_mcbsp->busy = 0;

	/* Init transfer tasklet */
	tasklet_init(&spi_mcbsp->pump_transfers,
		     spi_mcbsp_pump_transfers, (unsigned long)spi_mcbsp);

	/* Init messages workqueue */
	INIT_WORK(&spi_mcbsp->pump_messages, spi_mcbsp_pump_messages);
	spi_mcbsp->workqueue =
	        create_singlethread_workqueue(dev_name(spi_mcbsp->master->dev.parent));
	if (spi_mcbsp->workqueue == NULL)
		return -EBUSY;

	return 0;
}

static inline int start_queue(struct spi_mcbsp *spi_mcbsp)
{
	unsigned long flags;

	spin_lock_irqsave(&spi_mcbsp->lock, flags);

	if (spi_mcbsp->run == QUEUE_RUNNING || spi_mcbsp->busy) {
		spin_unlock_irqrestore(&spi_mcbsp->lock, flags);
		return -EBUSY;
	}

	spi_mcbsp->run          = QUEUE_RUNNING;
	spi_mcbsp->cur_msg      = NULL;
	spi_mcbsp->cur_transfer = NULL;
	spin_unlock_irqrestore(&spi_mcbsp->lock, flags);

	queue_work(spi_mcbsp->workqueue, &spi_mcbsp->pump_messages);

	return 0;
}

static inline int stop_queue(struct spi_mcbsp *spi_mcbsp)
{
	unsigned long flags;
	unsigned limit = 500;
	int status = 0;

	spin_lock_irqsave(&spi_mcbsp->lock, flags);

	/*
	 * This is a bit lame, but is optimized for the common execution path.
	 * A wait_queue on the spi_mcbsp->busy could be used, but then the common
	 * execution path (pump_messages) would be required to call wake_up or
	 * friends on every SPI message. Do this instead
	 */
	spi_mcbsp->run = QUEUE_STOPPED;
	while (!list_empty(&spi_mcbsp->queue) && spi_mcbsp->busy && limit--) {
		spin_unlock_irqrestore(&spi_mcbsp->lock, flags);
		msleep(10);
		spin_lock_irqsave(&spi_mcbsp->lock, flags);
	}

	if (!list_empty(&spi_mcbsp->queue) || spi_mcbsp->busy)
		status = -EBUSY;

	spin_unlock_irqrestore(&spi_mcbsp->lock, flags);

	return status;
}

static inline int destroy_queue(struct spi_mcbsp *spi_mcbsp)
{
	int status;

	status = stop_queue(spi_mcbsp);
	if (status != 0)
		return status;

	destroy_workqueue(spi_mcbsp->workqueue);

	return 0;
}

static void spi_mcbsp_cleanup(struct spi_device *spi)
{
	struct spi_mcbsp    *spi_mcbsp = get_spi_mcbsp(spi);
	struct mcbsp        *mcbsp     = spi_mcbsp->mcbsp;

	if (spi_mcbsp->first_setup_done) {
	        dev_dbg(spi_mcbsp->dev, "Freeing McBSP%d\n", get_mcbsp_id(spi));
       
		mcbsp_stop(get_mcbsp_id(spi));
		mcbsp_free(get_mcbsp_id(spi));
		
		edma_stop(mcbsp->dma_tx_lch);
		edma_stop(mcbsp->dma_rx_lch);
		
		edma_free_channel(mcbsp->dma_tx_lch);
		edma_free_channel(mcbsp->dma_rx_lch);
		
		spi_mcbsp->first_setup_done = 0;
	}
}

static int spi_mcbsp_probe(struct platform_device *pdev)
{
	struct device          *dev = &pdev->dev;
	struct spi_master      *master;
	struct spi_mcbsp       *spi_mcbsp;
	unsigned int            mcbsp_id;
	int			status = 0;

	master = spi_alloc_master(dev, sizeof *spi_mcbsp);
	if (!master)
		return -ENODEV;

	spi_mcbsp = spi_master_get_devdata(master);

	/* the spi->mode bits understood by this driver: */
	master->mode_bits      = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST;
	master->bus_num        = pdev->id;              /* McBSP instance */
	master->num_chipselect = (unsigned int) dev->platform_data;
	master->setup          = spi_mcbsp_setup;
	master->cleanup        = spi_mcbsp_cleanup;
	master->transfer       = spi_mcbsp_transfer;
	platform_set_drvdata(pdev, master);

	mcbsp_id               = master->bus_num;
	spi_mcbsp->master      = master;
	spi_mcbsp->dev         = dev;
	spi_mcbsp->mcbsp       = mcbsp_ptr[mcbsp_id];
	spi_mcbsp->mcbsp_id    = mcbsp_id;

	/* Initiate and start queue */
	status = init_queue(spi_mcbsp);
	if (status != 0) {
		dev_err(dev, "problem initializing queue\n");
		goto out_error_queue_alloc;
	}
	status = start_queue(spi_mcbsp);
	if (status != 0) {
		dev_err(dev, "problem starting queue\n");
		goto out_error_queue_alloc;
	}

	status = spi_register_master(master);
	if (status != 0) {
	        stop_queue(spi_mcbsp);
		dev_err(dev, "problem registering spi master\n");
		goto out_error_queue_alloc;
	}

	dev_dbg(dev, "McBSP%d interface probed successfully\n", pdev->id);

	return status;

out_error_queue_alloc:

	destroy_queue(spi_mcbsp);
	spi_master_put(master);
	
	return status;
}

static int spi_mcbsp_remove(struct device *dev)
{
	struct spi_mcbsp       *spi_mcbsp = dev_get_drvdata(dev);
	int			status = 0;

	if (!spi_mcbsp)
		return 0;

	/* Remove the queue */
	status = destroy_queue(spi_mcbsp);
	if (status != 0)
		return status;

	/* Disconnect from the SPI framework */
	spi_unregister_master(spi_mcbsp->master);

	/* Prevent double remove */
	dev_set_drvdata(dev, NULL);

	return status;
}

#ifdef CONFIG_PM

static int spi_mcbsp_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct spi_mcbsp  *spi_mcbsp= spi_master_get_devdata(master);

	clk_disable(spi_mcbsp->clk);
	return 0;
}

static int spi_mcbsp_resume(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct spi_mcbsp  *spi_mcbsp= spi_master_get_devdata(master);

	clk_enable(spi_mcbsp->clk);
	return 0;
}

#else
#define	spi_mcbsp_suspend	NULL
#define	spi_mcbsp_resume	NULL
#endif

MODULE_ALIAS("platform:spi_mcbsp");

static struct platform_driver spi_mcbsp_driver = {
	.driver		= {
		.name	= "spi_mcbsp",
		.owner	= THIS_MODULE,
	},
	.suspend	= spi_mcbsp_suspend,
	.resume		= spi_mcbsp_resume,
	.probe		= spi_mcbsp_probe,
	.remove		= __exit_p(spi_mcbsp_remove),
};

static int __init spi_mcbsp_init(void)
{
	int res;

	printk("SPI over McBSP driver version 0.1\n");

	res = platform_driver_register(&spi_mcbsp_driver);

	return res;
}

static void __exit spi_mcbsp_exit(void)
{
	platform_driver_unregister(&spi_mcbsp_driver);
}

module_init(spi_mcbsp_init);
module_exit(spi_mcbsp_exit);

MODULE_LICENSE("GPL");
