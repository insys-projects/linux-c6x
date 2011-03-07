/*
 *  arch/c6x/drivers/mcbsp-uart.c
 *
 *  UART bit-bang driver using McBSP
 *
 *  Copyright (C) 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <aurelien.jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/major.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/serial.h>

#include <asm/mcbsp.h>
#include <asm/mcbsp-uart.h>
#include <asm/edma.h>
#include <asm/gpio.h>

#ifdef DEBUG
#define DPRINTK(fmt, ARGS...)         do {				\
		printk("McBSP-UART: [%s]: " fmt, __FUNCTION__ , ## ARGS); \
	} while (0)
#else
#define DPRINTK( x... )
#endif

static unsigned int*       mcbsp_tx_buff[MAX_PORT];
static unsigned int*       mcbsp_rx_buff[MAX_PORT][2];
static struct edmacc_param dma_tx_params[MAX_PORT];

static struct uart_driver  serialmcbsp_serial_reg;

extern unsigned int        c6x_early_uart_cons;

static DEFINE_MUTEX(mcbsp_tx_mutex);
static DEFINE_MUTEX(mcbsp_mutex);

struct serialmcbsp_port {
	struct uart_port   port;      /* must be first */
        unsigned int       id;        /* McBSP ID used for this line */
	volatile int       stop;    
        unsigned int       running; 
        unsigned int       baudrate;
        unsigned int       rx_buff;
        struct tty_struct *tty;
        struct mcbsp      *mcbsp;
};

struct serialmcbsp_port    serialmcbsp_port[MAX_PORT];

#define	LINE(tty)	   ((tty)->index)
#define	PORT(tty)          ((struct serialmcbsp_port*) ((tty)->driver_data))

static struct mcbsp_reg_cfg initial_config = {
	.spcr2 = GRST | FRST | FREE,
	.spcr1 = 0,
	.rcr2  = RFIG,
	.rcr1  = RWDLEN1(MCBSP_WORD_32) | RFRLEN1(9),
	.xcr2  = RFIG,
	.xcr1  = RWDLEN1(MCBSP_WORD_32) | RFRLEN1(9),
	.srgr2 = 0,
	.srgr1 = 0,
	.pcr0  = CLKRP | FSXP | FSRP | FSXM | CLKXM | CLKRM,
};

static inline void mcbsp_get_char(struct serialmcbsp_port *port);

void mcbsp_edma_tx_callback(unsigned lch, u16 ch_status, void *data)
{
        struct serialmcbsp_port *port = (struct serialmcbsp_port *) (data);
	struct mcbsp            *mcbsp_dma = (struct mcbsp *) port->mcbsp;

	/* Wait transmit to be physically finished */
	while ((MCBSP_READ((int)mcbsp_ptr[port->id]->io_base, SPCR2) & XEMPTY));

	complete(&mcbsp_dma->tx_dma_completion);
}

void mcbsp_edma_rx_callback(unsigned lch, u16 ch_status, void *data)
{
        struct serialmcbsp_port *port = (struct serialmcbsp_port *) (data);

	mcbsp_get_char(port);
}

/*
 * Setting up EDMA Tx/Rx when opening the serial port
 */
static int mcbsp_edma_up(struct serialmcbsp_port *port, int rx_set)
{
        unsigned int           id = port->id;
        int                    dma_tx_ch, dma_tx_ch_reload;
        int                    dma_rx_ch, dma_rx_ch_reload;
	struct edmacc_param    tmp_params;
	
        /* 
	 * Setting up Tx EDMA channel configuration
	 */
	dma_tx_ch = edma_alloc_channel(mcbsp_ptr[id]->dma_tx_sync,
				       mcbsp_edma_tx_callback, port,
				       EVENTQ_4);
	if (dma_tx_ch < 0) {
	        DPRINTK("Unable to request DMA channel for McBSP%d Tx\n", id);
		return -EINVAL;
	}
	
	DPRINTK("Tx DMA on channel %d\n", dma_tx_ch);

	/* Request the dummy reload slot to avoid missed events */
	dma_tx_ch_reload = edma_alloc_slot(EDMA_CTLR(dma_tx_ch), EDMA_SLOT_ANY);
	if (dma_tx_ch_reload < 0) {
	        DPRINTK("Unable to request EDMA slot for McBSP%d Tx\n", id);
		return -EAGAIN;
	}
	
	mcbsp_ptr[id]->dma_tx_lch        = dma_tx_ch;
	mcbsp_ptr[id]->dma_tx_lch_reload = dma_tx_ch_reload;

	edma_stop(dma_tx_ch);
	
	edma_set_transfer_params(dma_tx_ch, 4, 1 * MCBSP_TX_BITS_PER_CHAR,
				 1, 0, ASYNC);
	
	edma_set_dest(dma_tx_ch, mcbsp_ptr[id]->dma_tx_data, 0, 0);
	edma_set_src(dma_tx_ch, (dma_addr_t) mcbsp_tx_buff[id], 0, 0);
	
	edma_set_src_index(dma_tx_ch, 4, 0);
	edma_set_dest_index(dma_tx_ch, 0, 0);

	edma_link(dma_tx_ch, dma_tx_ch_reload);

       	edma_read_slot(dma_tx_ch, &dma_tx_params[id]);

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
	
	if (!rx_set)
	        return 0;

	/* 
	 * Set up Rx EDMA channel configuration
	 */
	dma_rx_ch = edma_alloc_channel(mcbsp_ptr[id]->dma_rx_sync,
				       mcbsp_edma_rx_callback, port,
				       EVENTQ_4);
	if (dma_rx_ch < 0) {
	        DPRINTK("Unable to request DMA channel for McBSP%d Rx\n", id);
		return -EINVAL;
	}
	
	DPRINTK("Rx DMA on channel %d\n", dma_rx_ch);
	
	/* Request Rx EDMA reload param slot */
	dma_rx_ch_reload = edma_alloc_slot(EDMA_CTLR(dma_rx_ch),
					   EDMA_SLOT_ANY);
	if (dma_rx_ch_reload < 0) {
		DPRINTK("Unable to request DMA slot for McBSP%d Rx reload\n",
			id);
		return -EINVAL;
	}
	
	mcbsp_ptr[id]->dma_rx_lch        = dma_rx_ch;
	mcbsp_ptr[id]->dma_rx_lch_reload = dma_rx_ch_reload;

	edma_stop(dma_rx_ch);
	
	edma_set_transfer_params(dma_rx_ch, 4, MCBSP_MAX_RX_BAUD_BITS,
				 1, 1, ASYNC);

	edma_link(dma_rx_ch, dma_rx_ch_reload);

	edma_set_src(dma_rx_ch, mcbsp_ptr[id]->dma_rx_data, 0, 0);
	edma_set_dest(dma_rx_ch, 
		      (dma_addr_t)mcbsp_rx_buff[id][port->rx_buff], 0, 0);

	edma_set_src_index(dma_rx_ch, 0, 0);
	edma_set_dest_index(dma_rx_ch, 4, 0);

	/* Add transfer completion interrupt and restart of the channel */
	edma_read_slot(dma_rx_ch, &tmp_params);
	tmp_params.opt |= TCINTEN | EDMA_TCC(dma_rx_ch);
	edma_write_slot(dma_rx_ch, &tmp_params);
	edma_write_slot(dma_rx_ch_reload, &tmp_params);

	edma_set_dest(dma_rx_ch_reload,
		      (dma_addr_t)mcbsp_rx_buff[id][port->rx_buff ^ 1], 0, 0);

	/* Start Rx EDMA */
	edma_start(dma_rx_ch);

	/* Start Rx McBSP */
	mcbsp_start_rx(id);

	return 0;
}

static void mcbsp_edma_down(int id)
{
	edma_stop(mcbsp_ptr[id]->dma_tx_lch);
	edma_stop(mcbsp_ptr[id]->dma_rx_lch);

	edma_free_channel(mcbsp_ptr[id]->dma_tx_lch);
	edma_free_channel(mcbsp_ptr[id]->dma_rx_lch);
	edma_free_slot(mcbsp_ptr[id]->dma_rx_lch_reload);
	edma_free_slot(mcbsp_ptr[id]->dma_tx_lch_reload);

	mcbsp_ptr[id]->dma_tx_lch = -1;
	mcbsp_ptr[id]->dma_rx_lch = -1;
}

static inline void mcbsp_get_char(struct serialmcbsp_port *port)
{
        unsigned int       bit_idx;
	unsigned int	   size = MCBSP_MAX_RX_CHARS;
	char		   c;
	unsigned int       id  = port->id;
	struct tty_struct *tty = port->port.state->port.tty;

	L2_cache_block_invalidate((u32) mcbsp_rx_buff[id][port->rx_buff],
				  (u32) mcbsp_rx_buff[id][port->rx_buff] + (MCBSP_MAX_RX_BAUD_BITS * 4));

	/* Retrieve the 8 bits of the incomming character */
	for (bit_idx = 1, c = 0; bit_idx <= 8; bit_idx++)
	        c |= _extu(mcbsp_rx_buff[id][port->rx_buff][bit_idx], 16, 31) << (bit_idx - 1);
	/* Reset load parameters */
	edma_set_transfer_params(mcbsp_ptr[id]->dma_rx_lch_reload,
				 4, MCBSP_MAX_RX_BAUD_BITS,
				 1, 1, ASYNC);
	
	edma_set_dest(mcbsp_ptr[id]->dma_rx_lch_reload,
		      (dma_addr_t)mcbsp_rx_buff[id][port->rx_buff], 0, 0);

        /* Swap Rx buffers */
	port->rx_buff ^= 1; 

	if (uart_handle_sysrq_char(&port->port, c))
		goto ignore_char;

	while ((!port->stop) && size) {
  	        tty_insert_flip_char(tty, c, TTY_NORMAL);
		size--;
	}

ignore_char:

	if (size < MCBSP_MAX_RX_CHARS)
	        tty_flip_buffer_push(tty);
}

static inline void mcbsp_write(unsigned int id, const char* buf, unsigned int size)
{
        int           i, len;
	unsigned int *p_bit, bit_idx;
	char          mask, c;
	char         *b = (char *) buf;
	unsigned int  length;

	while(size > 0) {

	        if (size > MCBSP_MAX_TX_CHARS - 1) {
		        len = MCBSP_MAX_TX_CHARS;
		} else {
		        len = size;
		}
		size -= len;
		
		mutex_lock(&mcbsp_tx_mutex);

		/* Do UART 8N1 formatting of chars */
		for (i = 0, p_bit = mcbsp_tx_buff[id]; i < len; i++, b++) {
		        *p_bit++ = 0;            /* UART start bit */
			for (bit_idx = 0, c = *b, mask = 1 ; bit_idx < 8 ; bit_idx++, mask <<= 1)
			        *p_bit++ = (c & mask ? 0xFFFFFFFF : 0);
			*p_bit++ = 0xFFFFFFFF;
			*p_bit++ = 0xFFFFFFFF;   /* UART stop bit */
		}
		
		/* No coherency is assumed between EDMA and L2 cache */
		L2_cache_block_writeback((u32) mcbsp_tx_buff[id],
					 (u32) mcbsp_tx_buff[id] + (MCBSP_MAX_TX_BAUD_BITS * 4));
		
		length = len * MCBSP_TX_BITS_PER_CHAR;
		
		INIT_COMPLETION(mcbsp_ptr[id]->tx_dma_completion);

		edma_write_slot(mcbsp_ptr[id]->dma_tx_lch, &dma_tx_params[id]);
		edma_set_transfer_params(mcbsp_ptr[id]->dma_tx_lch, 4,
					 length, 1, 0, ASYNC);

		/* Start Tx EDMA */
		edma_start(mcbsp_ptr[id]->dma_tx_lch);

		/* Start McBSP Tx transmit and wait EDMA ending */
		mcbsp_start_tx(id);
		
		wait_for_completion(&(mcbsp_ptr[id]->tx_dma_completion));

		mcbsp_stop_tx(id);

		mutex_unlock(&mcbsp_tx_mutex);
	}
}

static inline void mcbsp_put_char(unsigned int id, const char c)
{
        mcbsp_write(id, &c, 1);
}

static int mcbsp_init(int id, unsigned int baudrate)
{
	struct clk *clk;
	int         clkgdv = 0;
	
	/*
	 *  Set Sample Rate at McBSP rate
	 */
	clk = mcbsp_get_clock(id);
	if (IS_ERR(clk)) {
		DPRINTK("cannot get McBSP device clock\n");
		return -ENODEV;
	} 

	/* Compute clkgdv based on McBSP clock and wanted baudrate */
	clkgdv = (clk_get_rate(clk) / baudrate / MMI_OVERSAMPLING) - 1;
	
	DPRINTK("clkgdv = %d\n", clkgdv);
	
	if ((clkgdv > 255) || (clkgdv < 0)) {
		/* 
		 * For requested sampling rate, the input clock to MCBSP cant be derived
		 * down to get the in range clock divider value for 16 bits sample
		 */
 	        DPRINTK("Invalid Baud Rate %d requested\n", (int) baudrate);
		return -EPERM;
	}
	
	initial_config.srgr1 = (FWID(MMI_OVERSAMPLING - 1) | CLKGDV(clkgdv));
	initial_config.srgr2 = (CLKSM | FPER(MMI_OVERSAMPLING * MCBSP_RX_BITS_PER_CHAR - 1));
	
	mcbsp_stop(id);
	mcbsp_config(id, &initial_config);
	
	if (mcbsp_tx_buff[id] == NULL)
		mcbsp_tx_buff[id] = (unsigned int *)
			kzalloc(max(MCBSP_MAX_TX_BAUD_BITS << 2, L2_CACHE_BYTES),
				GFP_KERNEL);
	
	if (mcbsp_rx_buff[id][0] == NULL)
		mcbsp_rx_buff[id][0] = (unsigned int *)
			kzalloc(max(MCBSP_MAX_RX_BAUD_BITS << 2, L2_CACHE_BYTES),
				GFP_KERNEL);
	
	if (mcbsp_rx_buff[id][1] == NULL)
		mcbsp_rx_buff[id][1] = (unsigned int *)
			kzalloc(max(MCBSP_MAX_RX_BAUD_BITS << 2, L2_CACHE_BYTES),
				GFP_KERNEL);
	
	return 0;
}

/*
 * Serial operations
 */
static inline void serialmcbsp_enable_line(int line) 
{    
#ifdef CONFIG_EVM6488_PCF857X
        if (line == 0) {
	        /* In case of the McBSP0, we need to set the CS on COM0 instead of SPI */
	        struct i2c_client *client;
		struct gpio_chip  *gpio;

		client = evm6488_i2c_get_client(NULL, PCF8515_I2C_ID);
		if (!client)
		        return;

		gpio = (struct gpio_chip *) i2c_get_clientdata(client);
		if (!gpio)
		        return;

	        /* Use the GPIO Expander to enable COM0 */
		gpio->set(gpio, 0, 1); /* COM0 PWR D8 */
		gpio->set(gpio, 1, 1); /* COM0 DR0/FSR0 D9 */
	}
#endif
}

static unsigned int serialmcbsp_tx_empty(struct uart_port *uport)
{
	return TIOCSER_TEMT;
}

static void serialmcbsp_set_mctrl(struct uart_port *uport, unsigned int mctrl)
{
}

static unsigned int serialmcbsp_get_mctrl(struct uart_port *uport)
{
	return 0;
}

static void serialmcbsp_stop_tx(struct uart_port *uport)
{
}

static void serialmcbsp_start_tx(struct uart_port *uport)
{
	struct serialmcbsp_port *port = (struct serialmcbsp_port *) uport;
	struct circ_buf *xmit = &uport->state->xmit;
	char tmp[MCBSP_MAX_TX_CHARS];
	int tosend;

	if (uport->x_char) {
		mcbsp_put_char(port->id, uport->x_char);
		uport->icount.tx++;
		uport->x_char = 0;
		return;
	}

	if (uart_tx_stopped(uport))
		return;

	while (!uart_circ_empty(xmit)) {
		tosend = 0;
		while (!uart_circ_empty(xmit) && tosend < sizeof(tmp)) {
			tmp[tosend++] = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			uport->icount.tx++;
		}
		mcbsp_write(port->id, tmp, tosend);
	};

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(uport);
}

static void serialmcbsp_stop_rx(struct uart_port *uport)
{
	struct serialmcbsp_port *port = (struct serialmcbsp_port *) uport;

	mcbsp_stop_rx(port->id);
}

static void serialmcbsp_break_ctl(struct uart_port *uport, int break_state)
{
}

static int serialmcbsp_startup(struct uart_port *uport)
{
	struct serialmcbsp_port *port = (struct serialmcbsp_port *) uport;
	struct tty_struct       *tty  = uport->state->port.tty;
	unsigned int baudrate;
	int res;

	mutex_lock(&mcbsp_mutex);

	baudrate = uart_get_baud_rate(uport, tty->termios, NULL,
				      MIN_BAUDRATE, MAX_BAUDRATE);

	DPRINTK("baudrate = %d\n", baudrate);
	
	port->stop           = 0;
	port->tty            = tty;
	port->baudrate       = baudrate;
	port->rx_buff        = 0;
	port->mcbsp          = mcbsp_ptr[port->id];
		
	if (port->running) {
		/* Case of an UART already used for the Linux console */
		mcbsp_stop(port->id);
		mcbsp_edma_down(port->id);
		port->running = 0;
	} else {
		/* New UART */
		if (mcbsp_request(port->id)) {
			mutex_unlock(&mcbsp_mutex);
			return -EBUSY;
		}
	}
		
	serialmcbsp_enable_line(port->id) ;
	
	res = mcbsp_init(port->id, baudrate);
	if (res) {
		mcbsp_free(port->id);
		mutex_unlock(&mcbsp_mutex);
		return res;
	}
		
	mcbsp_start_raw(port->id);
	
	res = mcbsp_edma_up(port, 1);
	if (res) {
		mcbsp_stop(port->id);
		mcbsp_free(port->id);
		mutex_unlock(&mcbsp_mutex);
		return res;
	}
		
	port->running = 1;
	
	if (tty_termios_baud_rate(tty->termios))
		tty_termios_encode_baud_rate(tty->termios, baudrate, baudrate);

	mutex_unlock(&mcbsp_mutex);
	
	return  0;
}

static void serialmcbsp_shutdown(struct uart_port *uport)
{
	struct serialmcbsp_port *port = (struct serialmcbsp_port *) uport;

	mutex_lock(&mcbsp_mutex);

	if (port->running) {
		mcbsp_stop(port->id);
		mcbsp_edma_down(port->id);
		mcbsp_free(port->id);
		port->running = 0;
	}

	mutex_unlock(&mcbsp_mutex);
}

static void
serialmcbsp_set_termios(struct uart_port *uport,
			struct ktermios *termios,
			struct ktermios *old)
{
	struct serialmcbsp_port *port = (struct serialmcbsp_port *) uport;
	unsigned int baudrate;
		
	mutex_lock(&mcbsp_mutex);

	baudrate = uart_get_baud_rate(uport, termios, old, MIN_BAUDRATE, MAX_BAUDRATE);

	DPRINTK("new baudrate = %d\n", baudrate);
		
	port->baudrate = baudrate;
	
	mcbsp_init(port->id, baudrate);
	
	mcbsp_start_raw(port->id);

	if (port->running)
		mcbsp_start_rx(port->id);

	mutex_unlock(&mcbsp_mutex);

	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baudrate, baudrate);
}

static void
serialmcbsp_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
}

static const char *
serialmcbsp_type(struct uart_port *port)
{
	return "McBSP-UART";
}

static void serialmcbsp_release_port(struct uart_port *port)
{
}

static int serialmcbsp_request_port(struct uart_port *port)
{
	return 0;
}

static void serialmcbsp_config_port(struct uart_port *port, int flags)
{
}

static int
serialmcbsp_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->baud_base < MIN_BAUDRATE || ser->type < PORT_UNKNOWN)
		return -EINVAL;
	return 0;
}

static void serialmcbsp_void_op(struct uart_port *port)
{
}

static struct uart_ops serialmcbsp_ops = {
	.tx_empty	= serialmcbsp_tx_empty,
	.set_mctrl	= serialmcbsp_set_mctrl,
	.get_mctrl	= serialmcbsp_get_mctrl,
	.stop_tx	= serialmcbsp_stop_tx,
	.start_tx	= serialmcbsp_start_tx,
	.stop_rx	= serialmcbsp_stop_rx,
	.enable_ms	= serialmcbsp_void_op,
	.break_ctl	= serialmcbsp_break_ctl,
	.startup	= serialmcbsp_startup,
	.shutdown	= serialmcbsp_shutdown,
	.set_termios	= serialmcbsp_set_termios,
	.set_ldisc	= serialmcbsp_void_op,
	.pm		= serialmcbsp_pm,
	.type		= serialmcbsp_type,
	.release_port	= serialmcbsp_release_port,
	.request_port	= serialmcbsp_request_port,
	.config_port	= serialmcbsp_config_port,
	.verify_port	= serialmcbsp_verify_port,
};

static int __devinit serialmcbsp_probe(struct platform_device *pdev)
{
	struct device           *dev  = get_device(&pdev->dev);
	struct mcbsp_uart_info  *info = (struct mcbsp_uart_info *) dev->platform_data;
	struct uart_port        *uport;
	struct serialmcbsp_port *port;
	int i;

	for (i = 0; i < info->mcbsp_num; i++) {
		int ret;

		port  = &serialmcbsp_port[i];
		uport = &port->port;

		memset(uport, 0, sizeof(struct uart_port));

		uport->type	= PORT_8250;    /* whatever */
		uport->uartclk	= 0;            /* not used */
		uport->dev	= dev;
		uport->ops	= &serialmcbsp_ops;
		uport->line      = i;

		ret = uart_add_one_port(&serialmcbsp_serial_reg, uport);
		if (ret < 0) {
			DPRINTK("uart_add_one_port() failed, ret = %d\n", ret);
			return ret;
		}

		/* Compute the McBSP ID used for this line */
		port->id = info->mcbsp_id + i;
	}
	return 0;
}

static int __devexit serialmcbsp_remove(struct platform_device *pdev)
{
	struct mcbsp_uart_info *info = (struct mcbsp_uart_info *) pdev->dev.platform_data;
	struct uart_port *port;
	int i;
	
	for (i = 0; i < info->mcbsp_num; i++) {
		port = &serialmcbsp_port[i].port;
		
		if (port->dev == &pdev->dev) {
			uart_remove_one_port(&serialmcbsp_serial_reg, port);
			port->dev = NULL;
			
	        if (mcbsp_tx_buff[i] != NULL)
		        kfree(mcbsp_tx_buff[i]);
		
		if (mcbsp_rx_buff[i][0] != NULL)
		        kfree(mcbsp_rx_buff[i][0]);
		
		if (mcbsp_rx_buff[i][1] != NULL)
		        kfree(mcbsp_rx_buff[i][1]);
		}
	}

	put_device(&pdev->dev);
	return 0;
}

static int serialmcbsp_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int serialmcbsp_resume(struct platform_device *dev)
{
	return 0;
}

static struct console serialmcbsp_serial_cons;

static struct uart_driver serialmcbsp_serial_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "mcbsp_serial_uart",
	.dev_name		= "ttySM",
	.major			= TTY_MAJOR,
	.minor			= MCBSP_TTY_MINORS,
	.nr			= MAX_PORT,
	.cons			= &serialmcbsp_serial_cons,
};

static struct platform_driver serialmcbsp_serial_driver = {
	.probe		= serialmcbsp_probe,
	.remove		= __devexit_p(serialmcbsp_remove),
	.suspend	= serialmcbsp_suspend,
	.resume		= serialmcbsp_resume,
	.driver		= {
		.name	= "mcbsp_serial",
		.owner	= THIS_MODULE,
	},
};

static int __init serialmcbsp_init(void)
{
	int ret;

	ret = uart_register_driver(&serialmcbsp_serial_reg);
	if (ret)
		goto out;

	ret = platform_driver_register(&serialmcbsp_serial_driver);
	if (ret == 0)
		goto out;

	uart_unregister_driver(&serialmcbsp_serial_reg);
out:
	return ret;
}

static void __exit serialmcbsp_fini(void)
{
	platform_driver_unregister(&serialmcbsp_serial_driver);
	uart_unregister_driver(&serialmcbsp_serial_reg);
}

module_init(serialmcbsp_init);
module_exit(serialmcbsp_fini);

#ifdef CONFIG_MCBSP_UART_CONSOLE

/*
 * Console management
 */
static int __init serialmcbsp_console_setup(struct console *c, char* options)
{
        struct serialmcbsp_port* port = serialmcbsp_port + (unsigned int) c->index;
	int baudrate = MAX_BAUDRATE;
	int bits     = 8;
	int parity   = 'n';
	int flow     = 'n';
	int res;

	if (options)
		uart_parse_options(options, &baudrate, &parity, &bits, &flow);
	
	res = uart_set_options(&port->port, c, baudrate, parity, bits, flow);
	if (res)
		return res;

	port->stop  = 0;
	port->mcbsp = mcbsp_ptr[port->id];

	if (mcbsp_request(port->id))
	        return -EBUSY;
	
	serialmcbsp_enable_line(c->index);

	res = mcbsp_init(port->id, baudrate);
	if (res)
	        return res;
	
	mcbsp_start_raw(port->id);

	mcbsp_stop_rx(port->id);

	udelay(1000);

	res = mcbsp_edma_up(port, 0);
	if (res) {
	        mcbsp_stop(port->id);
		return res;
	}
	
	port->baudrate = baudrate;
	port->running  = 1;

	return 1;
}

static void 
serialmcbsp_console_write(struct console* c, const char* buf, unsigned int size)
{
        struct serialmcbsp_port* port = serialmcbsp_port + (unsigned int) c->index;
	int i, idx;

	for (i = 0, idx = 0; i < size; i++) {
		if (buf[i] == '\n') {
		    mcbsp_write(port->id, &buf[idx], i - idx);
		    mcbsp_put_char(port->id, '\r');
		    mcbsp_put_char(port->id, buf[i]);
		    idx = i + 1; 
		}
	}
	mcbsp_write(port->id, &buf[idx], size - idx);
}

static struct console serialmcbsp_serial_cons = {
	.name                   = "ttySM",
	.device                 = uart_console_device,
	.write                  = serialmcbsp_console_write,
	.setup                  = serialmcbsp_console_setup,
	.flags                  = CON_ENABLED | CON_PRINTBUFFER,
	.index                  = -1,
	.data                   = &serialmcbsp_serial_reg,
};

static int __init serialmcbsp_console_init(void)
{
       register_console(&serialmcbsp_serial_cons);

       return 0;
}

/* We don't use console_initcall() as it is too early for McBSP */
late_initcall(serialmcbsp_console_init);
#endif
