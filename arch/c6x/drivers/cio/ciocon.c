/*
 * CIO Console Driver for use with TI simulators and emulators
 * Copyright (C) 2009, 2010 Texas Instruments, Incorparated (wmills@ti.com)
 *
 * Based on Tiny TTY driver from O'Reily "Linux Device Drivers" book examples
 * Copyright (C) 2002-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, version 2 of the License.
 *
 * This is a console and write only TTY driver for the C IO protocol which is
 * implemented by TI (and other) simulators and emulators.  C IO protocol is
 * normaly used between the tool (e.x. Code Composer Studio) and TI's C
 * run-time library.  C IO implements stdio to the tool's window and file I/O
 * to the host's systems file system.  In this context we limit to writes to
 * stdout and stderr only.  [CIO is a rather simplistic blocking protocol
 * and reads would block the whole cpu.]
 *
 * Updated for newer 2.6.x device model and now emulates a UART instead of
 * hooking into tty layer directly.
 *
 * Mark Salter <msalter@redhat.com>
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <asm/uaccess.h>
#include "linkage.h"
#include "trgdrv.h"

#define DRIVER_VERSION "v0.2"
#define DRIVER_AUTHOR "William Mills <wmills@ti.com>"
#define DRIVER_DESC "CIO Console driver (for use with simulators and emulators)"

/* Module information */
MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");

#define CIO_TTY_MAJOR		242	/* experimental range */
#define CIO_TTY_MINORS		1

/* in case its not obvious */
#define CIO_STDOUT		1
#define CIO_STDERR		2

static struct platform_device cio_devices = {
	.name	= "cio",
	.id	= 0,
};

/* forward reference */
static struct uart_driver cio_serial_reg;
static struct uart_ops cio_ops;

struct uart_cio_port {
	struct uart_port	port;
};

static struct uart_cio_port cio_table[CIO_TTY_MINORS];

/*
 * Register a set of CIO devices attached to a platform device.
 */
static int __devinit cio_probe(struct platform_device *dev)
{
	struct uart_cio_port *uart;
	int i;

	for (i = 0; i < CIO_TTY_MINORS; i++) {
		uart = &cio_table[i];

		memset(&uart->port, 0, sizeof(struct uart_port));

		uart->port.type		= PORT_8250; /* whatever */
		uart->port.uartclk	= 921600 * 16;
		uart->port.dev		= &dev->dev;
		uart->port.ops		= &cio_ops;

		uart_add_one_port(&cio_serial_reg, &uart->port);
	}
	return 0;
}

static void __init
cio_register_ports(struct uart_driver *drv, struct device *dev)
{
	int i;

	for (i = 0; i < CIO_TTY_MINORS; i++) {
		struct uart_cio_port *up = &cio_table[i];

		up->port.dev = dev;
		uart_add_one_port(drv, &up->port);
	}
}

static int __devexit cio_remove(struct platform_device *dev)
{
	int i;

	for (i = 0; i < CIO_TTY_MINORS; i++) {
		struct uart_cio_port *up = &cio_table[i];

		if (up->port.dev == &dev->dev) {
			uart_remove_one_port(&cio_serial_reg, &up->port);
			up->port.dev = NULL;
		}
	}
	return 0;
}

static int cio_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int cio_resume(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver cio_serial_driver = {
	.probe		= cio_probe,
	.remove		= __devexit_p(cio_remove),
	.suspend	= cio_suspend,
	.resume		= cio_resume,
	.driver		= {
		.name	= "cio",
		.owner	= THIS_MODULE,
	},
};

static int __init cio_init(void)
{
	int ret;

	ret = uart_register_driver(&cio_serial_reg);
	if (ret)
		goto out;

	cio_register_ports(&cio_serial_reg, &cio_devices.dev);

	ret = platform_driver_register(&cio_serial_driver);
	if (ret == 0)
		goto out;

	platform_device_del(&cio_devices);
	platform_device_put(&cio_devices);
	uart_unregister_driver(&cio_serial_reg);
out:
	return ret;
}

static void __exit cio_exit(void)
{
	platform_driver_unregister(&cio_serial_driver);
	platform_device_unregister(&cio_devices);

	uart_unregister_driver(&cio_serial_reg);
}

module_init(cio_init);
module_exit(cio_exit);

static void cio_write(int hfd, const char *s,
				unsigned count)
{
	int written;

	while (count > 0) {
		written = HOSTwrite(hfd, s, count);
		count  -= written;
		s      += written;
	}
}


static void cio_putchar(char ch)
{
	cio_write(CIO_STDOUT, &ch, 1);
}

/*
 *	Print a string to the serial port trying not to disturb
 *	any possible real use of the port...
 *
 *	The console must be locked when we get here.
 */
static void cio_console_write(struct console *co, const char *s, unsigned count)
{
	cio_write(CIO_STDOUT, s, count);
}

static struct console ciocons = {
	.name =		"cio",
	.device =	uart_console_device,
	.write =	cio_console_write,
	.flags =	CON_PRINTBUFFER,
	.index =	-1,
	.data =		&cio_serial_reg,
};

static struct uart_driver cio_serial_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "cio_serial",
	.dev_name		= "cio",
	.major			= CIO_TTY_MAJOR,
	.minor			= 0,
	.nr			= CIO_TTY_MINORS,
	.cons			= &ciocons,
};


static unsigned int cio_tx_empty(struct uart_port *port)
{
	return TIOCSER_TEMT;
}

static void cio_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int cio_get_mctrl(struct uart_port *port)
{
	return 0;
}

static void cio_void_op(struct uart_port *port)
{
}

static void cio_break_ctl(struct uart_port *port, int break_state)
{
}

static void cio_start_tx(struct uart_port *port)
{
	struct uart_cio_port *up = (struct uart_cio_port *)port;
	struct circ_buf *xmit = &up->port.state->xmit;
	char tmp[64];
	int tosend;

	if (up->port.x_char) {
		cio_putchar(up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_tx_stopped(&up->port))
		return;

	while (!uart_circ_empty(xmit)) {
		tosend = 0;
		while (!uart_circ_empty(xmit) && tosend < sizeof(tmp)) {
			tmp[tosend++] = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			up->port.icount.tx++;
		}
		cio_write(CIO_STDOUT, tmp, tosend);
	};

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	return;
}

static int cio_startup(struct uart_port *port)
{
	return 0;
}

static void
cio_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
}

static void
cio_pm(struct uart_port *port, unsigned int state, unsigned int oldstate)
{
}

static const char *cio_type(struct uart_port *port)
{
	return "cio";
}

static int cio_request_port(struct uart_port *port)
{
	return 0;
}

static void cio_config_port(struct uart_port *port, int flags)
{
}

static int cio_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	return 0;
}

static struct uart_ops cio_ops = {
	.tx_empty	= cio_tx_empty,
	.set_mctrl	= cio_set_mctrl,
	.get_mctrl	= cio_get_mctrl,
	.stop_tx	= cio_void_op,
	.start_tx	= cio_start_tx,
	.stop_rx	= cio_void_op,
	.enable_ms	= cio_void_op,
	.break_ctl	= cio_break_ctl,
	.startup	= cio_startup,
	.shutdown	= cio_void_op,
	.set_termios	= cio_set_termios,
	.set_ldisc	= cio_void_op,
	.pm		= cio_pm,
	.type		= cio_type,
	.release_port	= cio_void_op,
	.request_port	= cio_request_port,
	.config_port	= cio_config_port,
	.verify_port	= cio_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char = cio_get_poll_char,
	.poll_put_char = cio_put_poll_char,
#endif
};



/*
 *	Register console.
 */
static int __init cio_console_init(void)
{
	register_console(&ciocons);
	return 0;
}
console_initcall(cio_console_init);


/*
 *	Register device.
 */
static int __init cio_arch_init(void)
{
        return platform_device_register(&cio_devices);
}
arch_initcall(cio_arch_init);
