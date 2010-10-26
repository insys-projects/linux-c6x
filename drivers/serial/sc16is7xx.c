/*
 *  linux/drivers/serial/sc16is7xx.c
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Mark Salter <msalter@redhat.com>
 *
 *  Based on drivers/serial/8250.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2001 Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/tty.h>
#include <linux/serial_core.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c/sc16is7xx.h>
#include <linux/irq.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>

#include <asm/serial.h>

#define UART_NR		1
#define TX_LOADSZ	64
#define PASS_LIMIT	256
#define BOTH_EMPTY 	(UART_LSR_TEMT | UART_LSR_THRE)

static struct workqueue_struct *wq;

struct uart_sc16_port {
	struct uart_port	port;
	struct timer_list	timer;
	struct mutex		xfer_lock;
	struct work_struct	work;
	struct work_struct	backup_work;
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
#define LSR_SAVE_FLAGS UART_LSR_BRK_ERROR_BITS
	unsigned char		lsr_saved_flags;
#define MSR_SAVE_FLAGS UART_MSR_ANY_DELTA
	unsigned char		msr_saved_flags;

	unsigned char		mcr_mask;	/* mask of user bits */
	unsigned char		mcr_force;	/* mask of forced bits */
};

static const struct i2c_device_id sc16_ids[] = {
	{ "sc16is740", 1 },
	{ "sc16is750", 1 },
	{ "sc16is760", 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc_ids);

struct sc16_chip {
	struct i2c_client *client;
	struct sc16is7xx_platform_data *pdata;
	int ports[UART_NR];
	int nports;
};

static int serialsc16_register_port(struct uart_port *port);
static void serialsc16_unregister_port(int line);

static inline int i2c_write_reg(struct i2c_client *client, u8 reg, u8 data)
{
	int ret;
	u8 buf[] = { reg, data };
	struct i2c_msg msg = { .addr = client->addr, .flags = 0, .buf = buf, .len = 2 };

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1)
		printk(KERN_DEBUG "i2c_write_reg: error reg=0x%x, data=0x%x, ret=%i\n",
			reg, data, ret);

	return (ret != 1) ? -EIO : 0;
}

static inline int i2c_read_reg(struct i2c_client *client, u8 reg, u8 *p_data)
{
	int ret;

	u8 b0[] = { reg };
	u8 b1[] = { 0 };
	struct i2c_msg msg[] = {
		{ .addr = client->addr, .flags = 0, .buf = b0, .len = 1 },
		{ .addr = client->addr, .flags = I2C_M_RD, .buf = b1, .len = 1 },
	};

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		printk(KERN_DEBUG "i2c_read_reg: error reg=0x%x, ret=%i\n",
			reg, ret);
		return -EIO;
	}

	*p_data = b1[0];
	return 0;
}

static unsigned int sc16_serial_in(struct uart_port *port, int reg)
{
	struct sc16_chip *chip = port->private_data;
	int ret;
	u8 val;

	ret = i2c_read_reg(chip->client, (reg << 3) | (port->iobase << 1), &val);
	if (ret == 0)
		return val;

	printk(KERN_DEBUG "sc16_serial_in: i2c_read_reg failed!\n");
	return 0;
}

static void sc16_serial_out(struct uart_port *port, int reg, int val)
{
	struct sc16_chip *chip = port->private_data;
	int ret;

	ret = i2c_write_reg(chip->client, (reg << 3) | (port->iobase << 1), val);
	if (ret)
		printk(KERN_DEBUG "sc16_serial_out: i2c_write_reg failed!\n");
}

#define serial_in(up, offset)		\
	(up->port.serial_in(&(up)->port, (offset)))
#define serial_out(up, offset, value)	\
	(up->port.serial_out(&(up)->port, (offset), (value)))

static struct console serialsc16_console;

static int __devinit sc16_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct sc16is7xx_platform_data *pdata;
	struct sc16_chip *chip;
	struct uart_port port;
	int i, ret;

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		printk(KERN_DEBUG "sc16_probe: missing platform data!\n");
		return -EINVAL;
	}
	
	chip = kzalloc(sizeof(struct sc16_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->client = client;
	chip->pdata = pdata;
	chip->nports = ((id->driver_data & 0xFF) <= UART_NR) ?: UART_NR  ;

	i2c_set_clientdata(client, chip);

	memset(&port, 0, sizeof(struct uart_port));
	port.irq	= client->irq;
	port.flags	= UPF_FIXED_TYPE;
	port.uartclk	= pdata->baud_base * 16;
	port.type	= PORT_16IS740;
	port.iotype	= UPIO_PORT;
	port.dev	= &client->dev;
	port.serial_in	= sc16_serial_in;
	port.serial_out	= sc16_serial_out;
	port.private_data = chip;
	port.membase	= (unsigned char *)-1;

	for (i = 0; i < chip->nports; i++) {
		/* Use iobase field to hold our channel number. */
		port.iobase = i;
		chip->ports[i] = serialsc16_register_port(&port);
	}

	return 0;

	kfree(chip);
	return ret;
}

static int sc16_remove(struct i2c_client *client)
{
	struct sc16_chip *chip = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < chip->nports; i++)
		if (chip->ports[i] >= 0)
			serialsc16_unregister_port(chip->ports[i]);
	kfree(chip);
	
	return 0;
}

/* ================================================================= */

/* Uart divisor latch read */
static inline int serial_dl_read(struct uart_sc16_port *up)
{
	return serial_in(up, UART_DLL) | serial_in(up, UART_DLM) << 8;
}

/* Uart divisor latch write */
static inline void serial_dl_write(struct uart_sc16_port *up, int value)
{
	serial_out(up, UART_DLL, value & 0xff);
	serial_out(up, UART_DLM, value >> 8 & 0xff);
}

/*
 * FIFO support.
 */
static void serialsc16_clear_fifos(struct uart_sc16_port *p)
{
	serial_out(p, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_out(p, UART_FCR, UART_FCR_ENABLE_FIFO |
		    UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_out(p, UART_FCR, 0);
}

/*
 * IER sleep support.  The "extended capability" bit needs to be enabled
 * before modifying the sleep bit.
 */
static void serialsc16_set_sleep(struct uart_sc16_port *p, int sleep)
{
	serial_out(p, UART_EFR, UART_EFR_ECB);
	serial_out(p, UART_IER, sleep ? UART_IERX_SLEEP : 0);
	serial_out(p, UART_EFR, 0);
}

static inline void __stop_tx(struct uart_sc16_port *p)
{
	if (p->ier & UART_IER_THRI) {
		p->ier &= ~UART_IER_THRI;
		serial_out(p, UART_IER, p->ier);
	}
}

static void serialsc16_stop_tx(struct uart_port *port)
{
	struct uart_sc16_port *up = (struct uart_sc16_port *)port;

	__stop_tx(up);
}

static void serialsc16_start_tx(struct uart_port *port)
{
	struct uart_sc16_port *up = (struct uart_sc16_port *)port;

	if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
}

static void serialsc16_stop_rx(struct uart_port *port)
{
	struct uart_sc16_port *up = (struct uart_sc16_port *)port;

	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(up, UART_IER, up->ier);
}

static void serialsc16_enable_ms(struct uart_port *port)
{
	struct uart_sc16_port *up = (struct uart_sc16_port *)port;

	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
}

static void
receive_chars(struct uart_sc16_port *up, unsigned int *status)
{
	struct tty_struct *tty = up->port.state->port.tty;
	unsigned char ch, lsr = *status;
	int max_count = 256;
	char flag;

	do {
		if (likely(lsr & UART_LSR_DR))
			ch = serial_in(up, UART_RX);
		else
			/*
			 * Intel 82571 has a Serial Over Lan device that will
			 * set UART_LSR_BI without setting UART_LSR_DR when
			 * it receives a break. To avoid reading from the
			 * receive buffer without UART_LSR_DR bit set, we
			 * just force the read character to be 0
			 */
			ch = 0;

		flag = TTY_NORMAL;
		up->port.icount.rx++;

		lsr |= up->lsr_saved_flags;
		up->lsr_saved_flags = 0;

		if (uart_handle_sysrq_char(&up->port, ch))
			goto ignore_char;

		uart_insert_char(&up->port, lsr, UART_LSR_OE, ch, flag);

ignore_char:
		lsr = serial_in(up, UART_LSR);
	} while ((lsr & (UART_LSR_DR | UART_LSR_BI)) && (max_count-- > 0));
	tty_flip_buffer_push(tty);
	*status = lsr;
}

static void transmit_chars(struct uart_sc16_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int count;

	if (up->port.x_char) {
		serial_out(up, UART_TX, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_tx_stopped(&up->port)) {
		serialsc16_stop_tx(&up->port);
		return;
	}
	if (uart_circ_empty(xmit)) {
		__stop_tx(up);
		return;
	}

	count = TX_LOADSZ;
	do {
		serial_out(up, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit))
		__stop_tx(up);
}

static unsigned int check_modem_status(struct uart_sc16_port *up)
{
	unsigned int status = serial_in(up, UART_MSR);

	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;
	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    up->port.state != NULL) {
		if (status & UART_MSR_TERI)
			up->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			up->port.icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change(&up->port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
			uart_handle_cts_change(&up->port, status & UART_MSR_CTS);

		wake_up_interruptible(&up->port.state->port.delta_msr_wait);
	}

	return status;
}


static void sc16_do_work(struct uart_sc16_port *up)
{
	unsigned int status;

	mutex_lock(&up->xfer_lock);

	status = serial_in(up, UART_LSR);

	if (status & (UART_LSR_DR | UART_LSR_BI))
		receive_chars(up, &status);
	check_modem_status(up);
	if (status & UART_LSR_THRE)
		transmit_chars(up);

	mutex_unlock(&up->xfer_lock);
}

/*
 * Bottom half: handle the interrupt by receiving/transmitting chars
 */
static void sc16_work(struct work_struct *work)
{
	struct uart_sc16_port *up = container_of(work, struct uart_sc16_port, work);

	sc16_do_work(up);
	enable_irq(up->port.irq);
}

static void sc16_backup_work(struct work_struct *work)
{
	struct uart_sc16_port *up = container_of(work, struct uart_sc16_port, backup_work);

	sc16_do_work(up);
}

/*
 * Just start the workqueue thread to handle the interrupt
 */
static irqreturn_t serialsc16_interrupt(int irq, void *dev_id)
{
	struct uart_sc16_port *up = (struct uart_sc16_port *)dev_id;

	/* Acknowledge, clear *AND* mask the interrupt... */
	disable_irq_nosync(irq);
	queue_work(wq, &up->work);
	return IRQ_HANDLED;
}

/* Base timer interval for polling */
static inline int poll_timeout(int timeout)
{
	return timeout > 6 ? (timeout / 2 - 2) : 1;
}


static void serialsc16_backup_timeout(unsigned long data)
{
	struct uart_sc16_port *up = (struct uart_sc16_port *)data;
	
	queue_work(wq, &up->backup_work);

	/* Standard timer interval plus 0.2s to keep the port running */
	mod_timer(&up->timer,
		jiffies + poll_timeout(up->port.timeout) + HZ / 5);
}

static unsigned int serialsc16_tx_empty(struct uart_port *port)
{
	struct uart_sc16_port *up = (struct uart_sc16_port *)port;
	unsigned int lsr;

	mutex_lock(&up->xfer_lock);
	lsr = serial_in(up, UART_LSR);
	up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
	mutex_unlock(&up->xfer_lock);

	return (lsr & BOTH_EMPTY) == BOTH_EMPTY ? TIOCSER_TEMT : 0;
}

static unsigned int serialsc16_get_mctrl(struct uart_port *port)
{
	struct uart_sc16_port *up = (struct uart_sc16_port *)port;
	unsigned int status;
	unsigned int ret;

	status = check_modem_status(up);

	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void serialsc16_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_sc16_port *up = (struct uart_sc16_port *)port;
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr = (mcr & up->mcr_mask) | up->mcr_force | up->mcr;

	serial_out(up, UART_MCR, mcr);
}

static void serialsc16_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_sc16_port *up = (struct uart_sc16_port *)port;

	mutex_lock(&up->xfer_lock);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	mutex_unlock(&up->xfer_lock);
}

/*
 *	Wait for transmitter & holding register to empty
 */
static void wait_for_xmitr(struct uart_sc16_port *up, int bits)
{
	unsigned int status, tmout = 10000;

	/* Wait up to 10ms for the character(s) to be sent. */
	do {
		status = serial_in(up, UART_LSR);

		up->lsr_saved_flags |= status & LSR_SAVE_FLAGS;

		if (--tmout == 0)
			break;
		udelay(1);
	} while ((status & bits) != bits);
}

static int serialsc16_startup(struct uart_port *port)
{
	struct uart_sc16_port *up = (struct uart_sc16_port *)port;
	int retval;

	up->mcr = 0;

	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serialsc16_clear_fifos(up);

	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_in(up, UART_LSR);
	(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);

	/*
	 * Setup a backup timer
	 */
	up->timer.function = serialsc16_backup_timeout;
	up->timer.data = (unsigned long)up;
	mod_timer(&up->timer, jiffies +
		  poll_timeout(up->port.timeout) + HZ / 5);

	retval = request_irq(up->port.irq, serialsc16_interrupt,
			     0, "serial", up);
	if (retval < 0)
		return retval;

	/*
	 * Now, initialize the UART
	 */
	serial_out(up, UART_LCR, UART_LCR_WLEN8);

	mutex_lock(&up->xfer_lock);
	serialsc16_set_mctrl(&up->port, up->port.mctrl);
	mutex_unlock(&up->xfer_lock);

	/*
	 * Clear the interrupt registers again for luck, and clear the
	 * saved flags to avoid getting false values from polling
	 * routines or the previous session.
	 */
	serial_in(up, UART_LSR);
	serial_in(up, UART_RX);
	serial_in(up, UART_IIR);
	serial_in(up, UART_MSR);
	up->lsr_saved_flags = 0;
	up->msr_saved_flags = 0;

	/*
	 * Finally, enable interrupts.  Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	up->ier = UART_IER_RLSI | UART_IER_RDI;
	serial_out(up, UART_IER, up->ier);

	return 0;
}

static void serialsc16_shutdown(struct uart_port *port)
{
	struct uart_sc16_port *up = (struct uart_sc16_port *)port;

	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_out(up, UART_IER, 0);

	mutex_lock(&up->xfer_lock);
	serialsc16_set_mctrl(&up->port, up->port.mctrl);
	mutex_unlock(&up->xfer_lock);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(up, UART_LCR, serial_in(up, UART_LCR) & ~UART_LCR_SBC);
	serialsc16_clear_fifos(up);

	/*
	 * Read data port to reset things, and then unlink from
	 * the IRQ chain.
	 */
	(void) serial_in(up, UART_RX);

	del_timer_sync(&up->timer);
}

static void
serialsc16_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct uart_sc16_port *up = (struct uart_sc16_port *)port;
	unsigned char cval, fcr = 0, efr = 0;;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / 16 / 0xffff,
				  port->uartclk / 16);
	quot = uart_get_divisor(port, baud);

	if (baud < 2400)
		fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_1;
	else
		fcr = UART_FCR_ENABLE_FIFO | UART_FCR_R_TRIG_00 | UART_FCR_R_TRIG_10;

	/*
	 * Ok, we're now changing the port state.
	 */
	mutex_lock(&up->xfer_lock);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characteres to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * CTS flow control flag and modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;

	serial_out(up, UART_IER, up->ier);

	/*
	 * Hardware flow control.  FIXME:
	 * - TI16C752 requires control thresholds to be set.
	 * - UART_MCR_RTS is ineffective if auto-RTS mode is enabled.
	 */
	if (termios->c_cflag & CRTSCTS)
		efr |= UART_EFR_CTS;

	serial_out(up, UART_LCR, 0xBF);
	serial_out(up, UART_EFR, efr);
	serial_out(up, UART_LCR, cval | UART_LCR_DLAB);/* set DLAB */

	serial_dl_write(up, quot);

	serial_out(up, UART_LCR, cval);		/* reset DLAB */
	up->lcr = cval;					/* Save LCR */
	if (fcr & UART_FCR_ENABLE_FIFO)
		serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_out(up, UART_FCR, fcr);		/* set fcr */
	serialsc16_set_mctrl(&up->port, up->port.mctrl);
	mutex_unlock(&up->xfer_lock);
	/* Don't rewrite B0 */
	if (tty_termios_baud_rate(termios))
		tty_termios_encode_baud_rate(termios, baud, baud);
}

static void
serialsc16_set_ldisc(struct uart_port *port)
{
	int line = port->line;

	if (line >= port->state->port.tty->driver->num)
		return;

	if (port->state->port.tty->ldisc->ops->num == N_PPS) {
		port->flags |= UPF_HARDPPS_CD;
		serialsc16_enable_ms(port);
	} else
		port->flags &= ~UPF_HARDPPS_CD;
}

static void
serialsc16_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
	struct uart_sc16_port *p = (struct uart_sc16_port *)port;

	serialsc16_set_sleep(p, state != 0);
}

static void serialsc16_release_port(struct uart_port *port)
{
}

static int serialsc16_request_port(struct uart_port *port)
{
	return 0;
}

static void serialsc16_config_port(struct uart_port *port, int flags)
{
}

static int
serialsc16_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->irq >= nr_irqs || ser->irq < 0 ||
	    ser->baud_base < 9600 || ser->type < PORT_UNKNOWN)
		return -EINVAL;
	return 0;
}

static const char *
serialsc16_type(struct uart_port *port)
{
	return "NXP16IS7xx";
}

static struct uart_ops serialsc16_pops = {
	.tx_empty	= serialsc16_tx_empty,
	.set_mctrl	= serialsc16_set_mctrl,
	.get_mctrl	= serialsc16_get_mctrl,
	.stop_tx	= serialsc16_stop_tx,
	.start_tx	= serialsc16_start_tx,
	.stop_rx	= serialsc16_stop_rx,
	.enable_ms	= serialsc16_enable_ms,
	.break_ctl	= serialsc16_break_ctl,
	.startup	= serialsc16_startup,
	.shutdown	= serialsc16_shutdown,
	.set_termios	= serialsc16_set_termios,
	.set_ldisc	= serialsc16_set_ldisc,
	.pm		= serialsc16_pm,
	.type		= serialsc16_type,
	.release_port	= serialsc16_release_port,
	.request_port	= serialsc16_request_port,
	.config_port	= serialsc16_config_port,
	.verify_port	= serialsc16_verify_port,
};

static struct uart_sc16_port serialsc16_ports[UART_NR];

#ifdef CONFIG_SERIAL_SC16IS7XX_CONSOLE

static void serialsc16_console_putchar(struct uart_port *port, int ch)
{
	struct uart_sc16_port *up = (struct uart_sc16_port *)port;

	wait_for_xmitr(up, UART_LSR_THRE);
	serial_out(up, UART_TX, ch);
}

/*
 *	Print a string to the serial port trying not to disturb
 *	any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
void
serialsc16_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_sc16_port *up = &serialsc16_ports[co->index];
	unsigned int ier;
	int locked = 1;

	if (up->port.sysrq) {
		/* serialsc16_handle_port() already took the lock */
		locked = 0;
	} else if (oops_in_progress) {
		locked = mutex_trylock(&up->xfer_lock);
	} else
		mutex_lock(&up->xfer_lock);

	/*
	 *	First save the IER then disable the interrupts
	 */
	ier = serial_in(up, UART_IER);
	serial_out(up, UART_IER, 0);

	uart_console_write(&up->port, s, count, serialsc16_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	wait_for_xmitr(up, BOTH_EMPTY);
	serial_out(up, UART_IER, ier);

	/*
	 *	The receive handling will happen properly because the
	 *	receive ready bit will still be set; it is not cleared
	 *	on read.  However, modem control will not, we must
	 *	call it if we have saved something in the saved flags
	 *	while processing with interrupts off.
	 */
	if (up->msr_saved_flags)
		check_modem_status(up);

	if (locked)
		mutex_unlock(&up->xfer_lock);
}

static int __init serialsc16_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 9600;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	int ret;

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= UART_NR)
		co->index = 0;
	port = &serialsc16_ports[co->index].port;
	if (!port->iobase && !port->membase)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	ret = uart_set_options(port, co, baud, parity, bits, flow);
	return ret;
}

static struct uart_driver serialsc16_reg;

static struct console serialsc16_console = {
	.name		= "ttySI",
	.write		= serialsc16_console_write,
	.device		= uart_console_device,
	.setup		= serialsc16_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serialsc16_reg,
};

#define SERIALSC16_CONSOLE	&serialsc16_console
#else
#define SERIALSC16_CONSOLE	NULL
#endif

static struct uart_driver serialsc16_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "i2c_uart",
	.dev_name		= "ttySI",
	.major			= TTY_MAJOR,
	.minor			= 128,
	.cons			= SERIALSC16_CONSOLE,
	.nr			= UART_NR,
};

static int __init serialsc16_console_init(void)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_sc16_port *up = &serialsc16_ports[i];

		up->port.line = i;
		spin_lock_init(&up->port.lock);
		mutex_init(&up->xfer_lock);

		init_timer(&up->timer);
		up->timer.function = serialsc16_backup_timeout;

		INIT_WORK(&up->work, sc16_work);
		INIT_WORK(&up->backup_work, sc16_backup_work);

		up->port.ops = &serialsc16_pops;
	}

	register_console(&serialsc16_console);
	return 0;
}
console_initcall(serialsc16_console_init);


/*
 * serialsc16_register_port and serialsc16_unregister_port allows for
 * serial ports to be configured at run-time.
 */
static DEFINE_MUTEX(serial_mutex);

/**
 *	serialsc16_register_port - register a serial port
 *	@port: serial port template
 *
 *	Configure the serial port specified by the request. If the
 *	port exists and is in use, it is hung up and unregistered
 *	first.
 *
 *	The port is then probed and if necessary the IRQ is autodetected
 *	If this fails an error is returned.
 *
 *	On success the port is ready to use and the line number is returned.
 */
static int serialsc16_register_port(struct uart_port *port)
{
	struct uart_sc16_port *uart = NULL;
	int i, ret = -ENOSPC;

	if (port->uartclk == 0)
		return -EINVAL;

	mutex_lock(&serial_mutex);

	for (i = 0; i < UART_NR; i++)
		if (serialsc16_ports[i].port.type == PORT_UNKNOWN)
			uart = &serialsc16_ports[i];

	if (uart) {

		mutex_init(&uart->xfer_lock);
		init_timer(&uart->timer);

		uart->port.ops		= &serialsc16_pops;
		uart->port.type         = port->type;
		uart->port.iobase       = port->iobase;
		uart->port.membase      = port->membase;
		uart->port.irq          = port->irq;
		uart->port.irqflags     = port->irqflags;
		uart->port.uartclk      = port->uartclk;
		uart->port.fifosize     = port->fifosize;
		uart->port.regshift     = port->regshift;
		uart->port.iotype       = port->iotype;
		uart->port.flags        = port->flags | UPF_BOOT_AUTOCONF;
		uart->port.mapbase      = port->mapbase;
		uart->port.private_data = port->private_data;
		if (port->dev)
			uart->port.dev = port->dev;

		/* Possibly override default I/O functions.  */
		if (port->serial_in)
			uart->port.serial_in = port->serial_in;
		if (port->serial_out)
			uart->port.serial_out = port->serial_out;

		INIT_WORK(&uart->work, sc16_work);
		INIT_WORK(&uart->backup_work, sc16_backup_work);

		ret = uart_add_one_port(&serialsc16_reg, &uart->port);
		if (ret == 0)
			ret = uart->port.line;
	}
	mutex_unlock(&serial_mutex);

	return ret;
}

/**
 *	serialsc16_unregister_port - remove a 16x50 serial port at runtime
 *	@line: serial line number
 *
 *	Remove one serial port.  This may not be called from interrupt
 *	context.  We hand the port back to the our control.
 */
static void serialsc16_unregister_port(int line)
{
	struct uart_sc16_port *uart = &serialsc16_ports[line];

	mutex_lock(&serial_mutex);
	uart_remove_one_port(&serialsc16_reg, &uart->port);
	uart->port.dev = NULL;
	mutex_unlock(&serial_mutex);
}

static struct i2c_driver sc16_driver = {
	.driver = {
		.name = "sc16is7xx",
		.owner = THIS_MODULE,
	},
	.probe = sc16_probe,
	.remove = sc16_remove,
	.id_table = sc16_ids,
};

static int __init sc16_init(void)
{
	int ret;
	
	ret = uart_register_driver(&serialsc16_reg);
	if (ret)
		return ret;

	wq = create_singlethread_workqueue("sc16is7xx");
	if (!wq) {
		printk(KERN_ERR "sc16is7xx: workqueue FAIL\n");
		return -ESRCH;
	}

	return i2c_add_driver(&sc16_driver);
}
module_init(sc16_init);

static void __exit sc16_exit(void)
{
	i2c_del_driver(&sc16_driver);

	destroy_workqueue(wq);
	wq = NULL;
}
module_exit(sc16_exit);

MODULE_AUTHOR("Mark Salter <msalter@redhat.com");
MODULE_DESCRIPTION("NXP 8250-compatible serial port with I2C interface");
MODULE_LICENSE("GPL");
