/*
 *  linux/arch/c6x/platforms/board-evmtci6616.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/clk.h>
#include <linux/kernel_stat.h>
#include <linux/platform_device.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/timer.h>
#include <asm/percpu.h>
#include <asm/clock.h>

#include <mach/board.h>

SOC_CLK_DEF(122880000); /* SYSCLK is a 122.88 MHz clock */

static struct clk_lookup evm_clks[] = {
        SOC_CLK(),
	CLK("", NULL, NULL)
};

#ifdef CONFIG_EDMA3
#include <asm/edma.h>

static const s8
queue_tc_mapping0[][2] = {
	/* {event queue no, TC no} */
	{0, 0},
	{1, 1},
	{-1, -1},
};

static const s8
queue_priority_mapping0[][2] = {
	/* {event queue no, Priority} */
	{0, 0},
	{1, 1},
	{-1, -1},
};

static const s8
queue_tc_mapping1[][2] = {
	/* {event queue no, TC no} */
	{0, 0},
	{1, 1},
	{2, 2},
	{3, 3},
	{-1, -1},
};

static const s8
queue_priority_mapping1[][2] = {
	/* {event queue no, Priority} */
	{0, 0},
	{1, 1},
	{2, 2},
	{3, 3},
	{-1, -1},
};

static const s8
queue_tc_mapping2[][2] = {
	/* {event queue no, TC no} */
	{0, 0},
	{1, 1},
	{2, 2},
	{3, 3},
	{-1, -1},
};

static const s8
queue_priority_mapping2[][2] = {
	/* {event queue no, Priority} */
	{0, 0},
	{1, 1},
	{2, 2},
	{3, 3},
	{-1, -1},
};

static struct edma_soc_info edma_cc0_info = {
	.n_channel		= EDMA0_NUM_DMACH,
	.n_region		= EDMA_NUM_REGIONS,
	.n_slot			= EDMA0_NUM_PARAMENTRY,
	.n_tc			= EDMA0_NUM_EVQUE,
	.n_cc			= 3,
	.queue_tc_mapping	= queue_tc_mapping0,
	.queue_priority_mapping	= queue_priority_mapping0,
};

static struct edma_soc_info edma_cc1_info = {
	.n_channel		= EDMA_NUM_DMACH,
	.n_region		= EDMA_NUM_REGIONS,
	.n_slot			= EDMA1_NUM_PARAMENTRY,
	.n_tc			= EDMA1_NUM_EVQUE,
	.n_cc			= 3,
	.queue_tc_mapping	= queue_tc_mapping1,
	.queue_priority_mapping	= queue_priority_mapping1,
};

static struct edma_soc_info edma_cc2_info = {
	.n_channel		= EDMA_NUM_DMACH,
	.n_region		= EDMA_NUM_REGIONS,
	.n_slot			= EDMA2_NUM_PARAMENTRY,
	.n_tc			= EDMA2_NUM_EVQUE,
	.n_cc			= 3,
	.queue_tc_mapping	= queue_tc_mapping2,
	.queue_priority_mapping	= queue_priority_mapping2,
};

static struct edma_soc_info *edma_info[] = {
	&edma_cc0_info,
	&edma_cc1_info,
	&edma_cc2_info,
};

static struct resource edma_resources[] = {
	{
		.name	= "edma0",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "edma1",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "edma2",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "edma0_err",
		.start	= EDMA0_IRQ_CCERRINT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "edma1_err",
		.start	= EDMA1_IRQ_CCERRINT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "edma2_err",
		.start	= EDMA2_IRQ_CCERRINT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "edma_cc0",
		.start	= EDMA0_REGISTER_BASE,
		.end	= EDMA0_REGISTER_BASE + 0x7fff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma_cc1",
		.start	= EDMA1_REGISTER_BASE,
		.end	= EDMA1_REGISTER_BASE + 0x7fff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma_cc2",
		.start	= EDMA2_REGISTER_BASE,
		.end	= EDMA2_REGISTER_BASE + 0x7fff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma0_tc0",
		.start	= EDMA0_TC0_BASE,
		.end	= EDMA0_TC0_BASE + 0x03ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma0_tc1",
		.start	= EDMA0_TC1_BASE,
		.end	= EDMA0_TC1_BASE + 0x03ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma1_tc0",
		.start	= EDMA1_TC0_BASE,
		.end	= EDMA1_TC0_BASE + 0x03ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma1_tc1",
		.start	= EDMA1_TC1_BASE,
		.end	= EDMA1_TC1_BASE + 0x03ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma1_tc2",
		.start	= EDMA1_TC2_BASE,
		.end	= EDMA1_TC2_BASE + 0x03ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma1_tc3",
		.start	= EDMA1_TC3_BASE,
		.end	= EDMA1_TC3_BASE + 0x03ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma2_tc0",
		.start	= EDMA2_TC0_BASE,
		.end	= EDMA2_TC0_BASE + 0x03ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma2_tc1",
		.start	= EDMA2_TC1_BASE,
		.end	= EDMA2_TC1_BASE + 0x03ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma2_tc2",
		.start	= EDMA2_TC2_BASE,
		.end	= EDMA2_TC2_BASE + 0x03ff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "edma2_tc3",
		.start	= EDMA2_TC3_BASE,
		.end	= EDMA2_TC3_BASE + 0x03ff,
		.flags	= IORESOURCE_MEM,
	},
	/* not using TC*_ERR */
};

static struct platform_device edma_device = {
	.name			= "edma",
	.id			= 0,
	.dev.platform_data	= edma_info,
	.num_resources		= ARRAY_SIZE(edma_resources),
	.resource		= edma_resources,
};

static int __init evm_init_edma(void)
{
	int status;

	/* This is based on coreid, so must be calculated at runtime */
	edma_resources[0].start = EDMA0_IRQ_CCINT;
	edma_resources[1].start = EDMA1_IRQ_CCINT;
	edma_resources[2].start = EDMA2_IRQ_CCINT;

	status = platform_device_register(&edma_device);
	if (status != 0)
		pr_debug("setup_edma --> %d\n", status);

	return status;

}
arch_initcall(evm_init_edma);
#endif /* CONFIG_EDMA3 */

#ifdef CONFIG_SPI
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <mach/spi.h>

static struct spi_eeprom at25640b = {
	.byte_len	= 0x10000 / 8, /* 64Kbit 20Mhz EEPROM */
	.name		= "AT25640B",
	.page_size	= 32,
	.flags		= EE_ADDR2,    /* 16bit address */
};

static struct spi_board_info evm_spi_info[] __initconst = {
	{
		.modalias	= "at25",
		.platform_data	= &at25640b,
		.max_speed_hz	= 10 * 1000 * 1000,	/* 10 MHz at 3v3 */
		.bus_num	= 0,
		.chip_select	= 0,
		.mode		= SPI_MODE_0,
	},
};

static struct resource evm_spi0_resources[] = {
	{
		.start = SPI_REGISTER_BASE,
		.end   = SPI_REGISTER_END,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = IRQ_SPIINT0,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = EDMA_CTLR_CHAN(EDMA1_CTLR, DMA1_SPI_RX), /* Rx event */
		.flags = IORESOURCE_DMA,
	},
	{
		.start = EDMA_CTLR_CHAN(EDMA1_CTLR, DMA1_SPI_TX), /* Tx event */
		.flags = IORESOURCE_DMA,
	},
	{
		.start = EVENTQ_1,
		.flags = IORESOURCE_DMA,
	},
};

static struct davinci_spi_platform_data evm_spi0_pdata = {
	.version 	= SPI_VERSION_1,
	.num_chipselect = 1,
	.clk_internal	= 1,
	.cs_hold	= 1,
	.intr_level	= 0,
	.poll_mode	= 0, /* 0 -> interrupt mode, 1-> polling mode */
	.c2tdelay	= 0,
	.t2cdelay	= 0,
	.use_dma        = 1, /* use EDMA */
};

static struct platform_device evm_spi0_device = {
	.name = "spi_davinci",
	.id   = 0,
	.dev  = {
		.platform_data = &evm_spi0_pdata,
	},
	.num_resources = ARRAY_SIZE(evm_spi0_resources),
	.resource      = evm_spi0_resources,
};

static int __init board_setup_spi(void)
{
	platform_device_register(&evm_spi0_device);

	return spi_register_board_info(evm_spi_info, ARRAY_SIZE(evm_spi_info));
}
core_initcall(board_setup_spi);
#endif /* CONFIG_SPI */

#ifdef CONFIG_I2C
#ifdef CONFIG_EEPROM_AT24
static struct at24_platform_data at24_eeprom_data = {
       .byte_len       = 512 * 1024 / 8,
       .page_size      = 128,
       .flags          = AT24_FLAG_ADDR16,
};
#endif

static struct i2c_board_info evm_i2c_info[] = {
	{ I2C_BOARD_INFO("FPGA", 0x40), },
#ifdef CONFIG_EEPROM_AT24
	{ I2C_BOARD_INFO("24c512", 0x57),
	  .platform_data = &at24_eeprom_data,
	},
#endif
};

static int __init board_setup_i2c(void)
{
	return i2c_register_board_info(1, evm_i2c_info, ARRAY_SIZE(evm_i2c_info));
}
core_initcall(board_setup_i2c);

/*
 * FPGA support
 */
static struct i2c_client         *fpga_i2c_client = NULL;
static const struct i2c_device_id evm_fpga_ids[] = {
	{ "FPGA", 1 },
	{ }
};

static inline int fpga_i2c_write_reg(u8 reg, u8 data)
{
	struct i2c_client *client = fpga_i2c_client;
	int ret;
	u8 buf[] = { reg, data };
	struct i2c_msg msg = {
		.addr = client->addr, .flags = 0, .buf = buf, .len = 2
	};

	if (!client)
		return -ENODEV;
       

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1)
		printk(KERN_ERR "fpga_i2c_write_reg: error reg=0x%x, data=0x%x, ret=%i\n",
		       reg, data, ret);

	return (ret != 1) ? -EIO : 0;
}

static inline int fpga_i2c_read_reg(u8 reg, u8 *p_data)
{
	struct i2c_client *client = fpga_i2c_client;
	int ret;
	u8 b0[] = { reg };
	u8 b1[] = { 0 };
	struct i2c_msg msg[] = {
		{
			.addr = client->addr, .flags = 0,
			.buf = b0, .len = 1
		},
		{
			.addr = client->addr, .flags = I2C_M_RD,
			.buf = b1, .len = 1
		},
	};

	if (!client)
		return -ENODEV;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		printk(KERN_ERR "fpga_i2c_read_reg: error reg=0x%x, ret=%i\n",
			reg, ret);
		return -EIO;
	}

	*p_data = b1[0];

	return 0;
}

static int __devinit evm_fpga_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	fpga_i2c_client = client;

	return 0;
}

static struct i2c_driver evm_fpga_driver = {
	.driver = {
		.name = "FPGA",
		.owner = THIS_MODULE,
	},
	.probe    = evm_fpga_probe,
	.id_table = evm_fpga_ids,
};

/*
 * LEDs management
 */
static struct mutex	  leds_lock;
static struct work_struct leds_work;
static u8                 evm_leds_state[3]    = { -1, -1, -1 };
static u8                 evm_leds_started     = 0;
static u8                 evm_leds_initialized = 0;

static inline void evm_leds_set(unsigned int state, u8 reg, u8 shift)
{
	u8  val = 0;
	int res;

	mutex_lock(&leds_lock);

	res = fpga_i2c_read_reg(reg, &val);
	if (res) {
		mutex_unlock(&leds_lock);
		return;
	}

	val &= ~(0x3 << shift);
	val |= state << shift;

	(void) fpga_i2c_write_reg(reg, val);

	mutex_unlock(&leds_lock);
}

static void evm_leds_work(struct work_struct *work)
{
	if (unlikely(!evm_leds_initialized)) {
		/* Override LEDs */
		evm_leds_set(1, EVM_FPGA_LED_REG, EVM_FPGA_LEDO_S);

		/* Set initial LED settings */
		evm_leds_set(EVM_LED_YELLOW, EVM_FPGA_LED_REG, EVM_FPGA_LED1_S);
		evm_leds_set(EVM_LED_YELLOW, EVM_FPGA_LED_REG, EVM_FPGA_LED2_S);
		evm_leds_set(EVM_LED_GREEN,  EVM_FPGA_DSP_REG, EVM_FPGA_LED3_S);

		/* Intialization finished */
		evm_leds_initialized = 1;

	} else {
		/* Set hw LEDs status */
		if (evm_leds_state[0] != -1)
			evm_leds_set(evm_leds_state[0], EVM_FPGA_LED_REG, EVM_FPGA_LED1_S);

		if (evm_leds_state[1] != -1)
			evm_leds_set(evm_leds_state[1], EVM_FPGA_LED_REG, EVM_FPGA_LED2_S);
		
		if (evm_leds_state[2] != -1)
			evm_leds_set(evm_leds_state[2], EVM_FPGA_DSP_REG, EVM_FPGA_LED3_S);
	}
}

#ifdef CONFIG_IDLE_LED
#include <linux/timer.h>
static struct timer_list leds_timer;

/*
 * Timer LED
 */
static void evm_leds_timer(unsigned long dummy) 
{
	static unsigned char leds = 0;

	mod_timer(&leds_timer, jiffies + msecs_to_jiffies(250));

	leds = (~leds) & 1;

	if (leds)
		evm_leds_state[EVM_LED_TIMER_NUM] = EVM_LED_GREEN;
	else
		evm_leds_state[EVM_LED_TIMER_NUM] = EVM_LED_OFF;

	if (likely(evm_leds_started))
		schedule_work(&leds_work);
}

/*
 * Idle LED blink
 */
void c6x_arch_idle_led(int state)
{
#if 0
	if (state)
		evm_leds_state[EVM_LED_IDLE_NUM] = EVM_LED_RED;
	else
		evm_leds_state[EVM_LED_IDLE_NUM] = EVM_LED_OFF;

	if (likely(evm_leds_started))
		schedule_work(&leds_work);
#endif
}

#endif /* CONFIG_IDLE_LED */

static int __init evm_init_leds(void)
{
	INIT_WORK(&leds_work, evm_leds_work);

	mutex_init(&leds_lock);

	/* LEDs can be used now  */
	evm_leds_started = 1;

#ifdef CONFIG_IDLE_LED
	init_timer(&leds_timer);
	leds_timer.function = evm_leds_timer;
	mod_timer(&leds_timer, jiffies + msecs_to_jiffies(250));
#endif

	return 0;
}

static int __init evm_init_fpga(void)
{
	int res;

	res = i2c_add_driver(&evm_fpga_driver);
	if (res < 0)
		return res;

	/* Initialize LEDs support */
	return evm_init_leds();
}
device_initcall(evm_init_fpga);
#endif /* CONFIG_I2C */

#if defined(CONFIG_MTD_NAND_GPIO_C6X) || defined(CONFIG_MTD_NAND_GPIO_C6X_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand-gpio-c6x.h>
static struct mtd_partition evm_nand_parts[] = {
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= 0x00200000,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x01000000,
		.mask_flags	= 0,
	},
	{
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}	
};

static struct gpio_nand_platdata evm_nand_platdata = {
	.parts = evm_nand_parts,
	.num_parts = ARRAY_SIZE(evm_nand_parts),
	.chip_delay = 25,
};

static struct platform_device evm_nand = {
	.name		= "gpio-nand-c6x",
	.id		= -1,
	.dev		= {
		.platform_data = &evm_nand_platdata,
	}
};

static int __init evm_setup_nand(void)
{
	return platform_device_register(&evm_nand);
}
core_initcall(evm_setup_nand);
#endif

#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_SERIAL_8250_MODULE)
#include <linux/serial_8250.h>
#include <linux/serial.h>
/*
 * Configuration for 16550-like UART
 */
static struct plat_serial8250_port serial8250_platform_data [] = {
        {
                .membase  = (void *) UART_BASE_ADDR,
                .mapbase  = UART_BASE_ADDR,
                .irq      = IRQ_UART,
                .flags    = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
                .iotype   = UPIO_MEM32,
                .regshift = 2,
        },
        {
                .flags          = 0
        },
};

static struct platform_device serial8250_device = {
        .id             = PLAT8250_DEV_PLATFORM,
        .name           = "serial8250",
        .dev            = {
                .platform_data = serial8250_platform_data,
        },
};

#include <linux/serial_reg.h>
#define SERIAL_OUT(offset, value) writel(value, UART_BASE_ADDR + ((offset) << 2))

static int __init evm_init_uart(void)
{
	struct clk *clk;

	/* 
	 * Reset and enable the UART peripheral
	 */
	SERIAL_OUT(0xc, 0x0000);    /* Reset the UART */
	udelay(100);
	SERIAL_OUT(0xc, 0x6001);    /* UTRST | URRST | FREE */
	SERIAL_OUT(UART_FCR, 0x8);  /* DMAMODE1 */

	/*
	 *  Retrieve the UART clock
	 */
	clk = clk_get(NULL, "uart");
	if (IS_ERR(clk))
		return -ENODEV;
	else
		serial8250_platform_data[0].uartclk = clk_get_rate(clk);

	return platform_device_register(&serial8250_device);
}
core_initcall(evm_init_uart);
#endif

static void dummy_print_dummy(char *s, unsigned long hex) {}
static void dummy_progress(unsigned int step, char *s) {}

/* Called from arch/kernel/setup.c */
void c6x_board_setup_arch(void)
{   
	printk("Designed for the EVMTCI6616 board, Texas Instruments.\n");

	mach_progress      = dummy_progress;
	mach_print_value   = dummy_print_dummy;

	c6x_clk_init(evm_clks);

	mach_progress(1, "End of EVMTCI6616 specific initialization");
}

__init int evm_init(void)
{
	return 0;
}

arch_initcall(evm_init);
