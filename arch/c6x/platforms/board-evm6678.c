/*
 *  linux/arch/c6x/platforms/board-evm6678.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated
 *  Author: Sandeep Paulraj <s-paulraj@ti.com>
 *          Aurelien Jacquiot <a-jacquiot@ti.com>
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
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/timer.h>
#include <asm/percpu.h>
#include <asm/clock.h>
#include <asm/edma.h>

#include <mach/board.h>

SOC_CLK_DEF(100000000); /* SYSCLK is a 100 MHz clock */

static struct clk_lookup evm_clks[] = {
        SOC_CLK(),
	CLK("", NULL, NULL)
};

#ifdef CONFIG_EDMA3

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
	.n_region		= EDMA0_NUM_REGIONS,
	.n_slot			= EDMA0_NUM_PARAMENTRY,
	.n_tc			= EDMA0_NUM_TC,
	.n_cc			= 3,
	.queue_tc_mapping	= queue_tc_mapping0,
	.queue_priority_mapping	= queue_priority_mapping0,
};

static struct edma_soc_info edma_cc1_info = {
	.n_channel		= EDMA1_NUM_DMACH,
	.n_region		= EDMA1_NUM_REGIONS,
	.n_slot			= EDMA1_NUM_PARAMENTRY,
	.n_tc			= EDMA1_NUM_TC,
	.n_cc			= 3,
	.queue_tc_mapping	= queue_tc_mapping1,
	.queue_priority_mapping	= queue_priority_mapping1,
};

static struct edma_soc_info edma_cc2_info = {
	.n_channel		= EDMA2_NUM_DMACH,
	.n_region		= EDMA2_NUM_REGIONS,
	.n_slot			= EDMA2_NUM_PARAMENTRY,
	.n_tc			= EDMA2_NUM_TC,
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

#if defined(CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
#include <mach/emif.h>
#include <mach/nand.h>

static struct mtd_partition evm6678_nand_parts[] = {
	{
		.name		= "bootconfig",
		.offset		= 0,
		.size		= 0x4000,
		.mask_flags	= 0,
	},
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x00FFC000,
		.mask_flags	= 0,
	},
	{
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
};

static struct davinci_nand_pdata evmc6678_nand_data = {
	.mask_cle 		= 0x4000,
	.mask_ale 		= 0x2000,
	.parts			= evm6678_nand_parts,
	.nr_parts		= ARRAY_SIZE(evm6678_nand_parts),
	.ecc_mode		= NAND_ECC_HW,
	.ecc_bits               = 4,
	.options	        = 0,
};

static struct resource evmc6678_nand_resources[] = {
	{
		.start		= RAM_EMIF_CE2,
		.end		= RAM_EMIF_CE2 + 0x3FFFFFF,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= EMIF_BASE,
		.end		= EMIF_BASE + 0xFF,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device evmc6678_nand_device = {
	.name			= "davinci_nand",
	.id			= 0,

	.num_resources		= ARRAY_SIZE(evmc6678_nand_resources),
	.resource		= evmc6678_nand_resources,

	.dev			= {
		.platform_data	= &evmc6678_nand_data,
	},
};

static int __init evm_init_nand(void)
{
	return platform_device_register(&evmc6678_nand_device);
}
core_initcall(evm_init_nand);
#endif

#ifdef CONFIG_SPI
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/spi/eeprom.h>
#include <mach/spi.h>

/*
 * FPGA support
 */
static struct spi_device *fpga_spi_client;
static struct mutex	  fpga_lock;
static struct work_struct fpga_work;

static inline int spi_rw(struct spi_device *spi, u8 *tx_buf, u8 *rx_buf, size_t len)
{
	struct spi_transfer t = {
		.tx_buf		= tx_buf,
		.rx_buf		= rx_buf,
		.len		= len,
	};
	struct spi_message  m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(spi, &m);
}

static inline int fpga_spi_write_reg(u8 reg, u8 data)
{
	u16 buf;
	u16 dummy = 0;

	buf = ((EVM_FPGA_WR_CMD | reg) << 8) | data;

	return spi_rw(fpga_spi_client, (u8*) &buf, (u8*) &dummy, 2);
}

static inline int fpga_spi_read_reg(u8 reg, u8 *p_data)
{
	u16 cmd = ((EVM_FPGA_RD_CMD | reg) << 8);
	u16 buf = 0;
	int res;

	res = spi_rw(fpga_spi_client, (u8*) &cmd, (u8*) &buf, 2);

	if (res < 0)
		return res;

	*p_data = (u8) buf & 0xff;

	return res;
}

static inline void evm_fpga_set(u8 state, u8 reg, u8 shift)
{
	u8  val = 0;
	int res;

	mutex_lock(&fpga_lock);

	res = fpga_spi_read_reg(reg, &val);
	if (res < 0) {
		mutex_unlock(&fpga_lock);
		return;
	}

	val &= ~(1 << shift);
	val |= (state & 0x1) << shift;

	(void) fpga_spi_write_reg(reg, val);

	mutex_unlock(&fpga_lock);
}

static int __init evm_init_leds(void);

static void evm_fpga_work(struct work_struct *work)
{
	/* Disable NOR write protect */
	evm_fpga_set(0, EVM_FPGA_MISC_REG, EVM_FPGA_MISC_NOR_WP);

	/* Disable NAND write protect */
	evm_fpga_set(1, EVM_FPGA_MISC_REG, EVM_FPGA_MISC_NAND_WP);

	/* Disable EEPROM write protect */
	evm_fpga_set(0, EVM_FPGA_MISC_REG, EVM_FPGA_MISC_EEPROM_WP);

	/* Initialize LEDs support */
	(void) evm_init_leds();
}

static int __devinit evm_fpga_probe(struct spi_device *spi)
{
	fpga_spi_client = spi;

	INIT_WORK(&fpga_work, evm_fpga_work);

	mutex_init(&fpga_lock);

 	schedule_work(&fpga_work);

	return 0;
}

static struct spi_driver evm_fpga_driver = {
	.driver = {
		.name	= "FPGA",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	        = evm_fpga_probe,
};

static int __init evm_init_fpga(void)
{
	return spi_register_driver(&evm_fpga_driver);
}
device_initcall(evm_init_fpga);

/*
 * LEDs management
 */
static struct work_struct leds_work;
static char               evm_leds_state[4]    = { -1, -1, -1, -1 };
static u8                 evm_leds_started     = 0;
static u8                 evm_leds_initialized = 0;

static void evm_leds_work(struct work_struct *work)
{
	if (unlikely(!evm_leds_initialized)) {
		/* Set initial LED settings */
		evm_fpga_set(EVM_LED_ON, EVM_FPGA_LED_REG, EVM_FPGA_LED1);
		evm_fpga_set(EVM_LED_ON, EVM_FPGA_LED_REG, EVM_FPGA_LED2);
		evm_fpga_set(EVM_LED_ON, EVM_FPGA_LED_REG, EVM_FPGA_LED3);
		evm_fpga_set(EVM_LED_ON, EVM_FPGA_LED_REG, EVM_FPGA_LED4);

		/* Intialization finished */
		evm_leds_initialized = 1;

	} else {
		/* Set hw LEDs status */
		if (evm_leds_state[0] != -1)
			evm_fpga_set(evm_leds_state[0], EVM_FPGA_LED_REG, EVM_FPGA_LED1);

		if (evm_leds_state[1] != -1)
			evm_fpga_set(evm_leds_state[1], EVM_FPGA_LED_REG, EVM_FPGA_LED2);
		
		if (evm_leds_state[2] != -1)
			evm_fpga_set(evm_leds_state[2], EVM_FPGA_LED_REG, EVM_FPGA_LED3);

		if (evm_leds_state[3] != -1)
			evm_fpga_set(evm_leds_state[3], EVM_FPGA_LED_REG, EVM_FPGA_LED4);
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
		evm_leds_state[EVM_LED_TIMER_NUM] = EVM_LED_ON;
	else
		evm_leds_state[EVM_LED_TIMER_NUM] = EVM_LED_OFF;

	if (likely(evm_leds_started))
		schedule_work(&leds_work);
}

/*
 * Idle LED blink: LEDs are async due to SPI thus this feature cannot be impemented
 */
void c6x_arch_idle_led(int state) {}

#endif /* CONFIG_IDLE_LED */

static int __init evm_init_leds(void)
{
	INIT_WORK(&leds_work, evm_leds_work);

	/* LEDs can be used now  */
	evm_leds_started = 1;

#ifdef CONFIG_IDLE_LED
	init_timer(&leds_timer);
	leds_timer.function = evm_leds_timer;
	mod_timer(&leds_timer, jiffies + msecs_to_jiffies(250));
#endif
	return 0;
}

/*
 * SPI NOR Flash support
 */
static struct mtd_partition n25q128_evm_partitions[] = {
	{
		.name		= "test",
		.offset		= 0,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
};

static const struct flash_platform_data n25q128 = {
	.type           = "n25q128",
	.name		= "spi_flash",
	.parts		= n25q128_evm_partitions,
	.nr_parts	= ARRAY_SIZE(n25q128_evm_partitions),
};

/*
 * SPI
 */
static struct spi_board_info evm_spi_info[] __initconst = {
#ifndef CONFIG_IDLE_LED
	/*
	 * SPI access to the FPGA for LED blinking is today incompatible 
	 * with the NOR flash access. So set them exclusive.
	 */
	{
		.modalias	= "m25p80",
		.platform_data	= &n25q128,
		.max_speed_hz	= 25000000,
		.bus_num	= 0,
		.chip_select	= 0,
		.mode		= SPI_MODE_0,
		.bits_per_word  = 8,
	},
#endif
	{
		.modalias	= "FPGA",
		.platform_data	= NULL,
		.max_speed_hz	= 25000000,
		.bus_num	= 0,
		.chip_select	= 1,
		.mode		= SPI_MODE_1,
		.bits_per_word  = 16,
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
	.num_chipselect = 2,
	.clk_internal	= 1,
	.cs_hold	= 0,
	.intr_level	= 0,
	.poll_mode	= 0, /* 0 -> interrupt mode, 1-> polling mode */
	.c2tdelay	= 0,
	.t2cdelay	= 0,
	.use_dma        = 0, /* do not use EDMA */
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
	.byte_len	= 1024 * 1024 / 8,
	.page_size	= 256,
	.flags		= AT24_FLAG_ADDR16,
};
#endif

static struct i2c_board_info evm_i2c_info[] = {
#ifdef CONFIG_EEPROM_AT24
	{ I2C_BOARD_INFO("24c1024", 0x50),
	  .platform_data = &at24_eeprom_data,
	},
#endif
};

static int __init board_setup_i2c(void)
{
	return i2c_register_board_info(1, evm_i2c_info, ARRAY_SIZE(evm_i2c_info));
}
core_initcall(board_setup_i2c);
#endif /* CONFIG_I2C */

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
                .irq      = IRQ_UARTINT,
                .flags    = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
                .iotype   = UPIO_MEM32,
                .regshift = 2,
        },
        {
                .flags    = 0,
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

#ifdef CONFIG_TI_KEYSTONE_RAPIDIO
#include <linux/rio.h>
#include <linux/keystone/rio.h>
#include <mach/keystone_qmss.h>
/* 
 * SerDes and port mode configurations for different sRIO modes.
 * All configurations are based on a 156.25 MHz SerDes reference clock,
 * mode 0 is the default sRIO boot mode of the board.
 */
static struct keystone_serdes_config evm6678_serdes_config[4]  = {
    /* sRIO config 0: MPY = 10x, div rate = half, link rate = 3.125 Gbps, mode 1x */
	{ 0x0c053860, /* VBUS freq 313 - 626, promiscuous */
      0x0251,
      0x0010,
	  { 0x00440495, 0x00440495, 0x00440495, 0x00440495 },
	  { 0x00180795, 0x00180795, 0x00180795, 0x00180795 },
	  { 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
    /* sRIO config 1: MPY = 10x, div rate = half, link rate = 3.125 Gbps, mode 4x */
	{ 0x0c053860, /* VBUS freq 313 - 626, promiscuous */
      0x0251,
      0x0010,
	  { 0x00440495, 0x00440495, 0x00440495, 0x00440495 },
	  { 0x00180795, 0x00180795, 0x00180795, 0x00180795 },
	  { 0x00000004, 0x00000004, 0x00000004, 0x00000004 } },
    /* sRIO config 2: MPY = 8x, div rate = half, link rate = 2.5 Gbps, mode 1x */
	{ 0x0c053860, /* VBUS freq 313 - 626, promiscuous */
      0x0241,
      0x0010,
	  { 0x00440495, 0x00440495, 0x00440495, 0x00440495 },
	  { 0x00180795, 0x00180795, 0x00180795, 0x00180795 },
      { 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
    /* sRIO config 3: MPY = 8x, div rate = half, link rate = 2.5 Gbps, mode 4x */
	{ 0x0c053860, /* VBUS freq 313 - 626, promiscuous */
      0x0241,
      0x0010,
	  { 0x00440495, 0x00440495, 0x00440495, 0x00440495 },
	  { 0x00180795, 0x00180795, 0x00180795, 0x00180795 },
	  { 0x00000004, 0x00000004, 0x00000004, 0x00000004 } },
};

static struct keystone_rio_board_controller_info evm6678_rio_controller = {
	0xf,                         /* bitfield of port(s) to probe on this controller */
	0,                           /* default SerDes configuration */
	0,                           /* host id */
	RIO_DO_ENUMERATION,          /* initialisation method */
	0,                           /* 8bit ID size */
	evm6678_serdes_config,       /* SerDes configurations */
	ARRAY_SIZE(evm6678_serdes_config), /* number of SerDes configurations */
	RIO_PACKET_TYPE_MESSAGE,     /* default outbound mbox packet type */
	4,                           /* number of mbox */
	DEVICE_QM_PEND17,            /* RXU queues */
	DEVICE_QM_PEND21,            /* TXU completion queue */
	IRQ_QMPEND17,                /* RXU interrupts */
	IRQ_QMPEND21,                /* TXU interrupt */
};

static struct platform_device evm6678_rio_device = {
	.name           = "keystone-rapidio",
	.id             = 1,
	.dev		= {
		.platform_data = &evm6678_rio_controller,
	},
};

static int __init evm_init_rio(void)
{
	if (get_coreid() >> 2) {
		/*
		 * On Shannon, interrupt mapping is different for CP_INTC1 (core 4 to 7),
		 * we use here same IRQs but associated QPEND are shifted of 6.
		 */
		evm6678_rio_controller.rxu_queues = DEVICE_QM_PEND23;
		evm6678_rio_controller.txu_queue  = DEVICE_QM_PEND27;
	}
	return platform_device_register(&evm6678_rio_device);
}

core_initcall(evm_init_rio);
#endif /* CONFIG_TI_KEYSTONE_RAPIDIO */

static void dummy_print_dummy(char *s, unsigned long hex) {}
static void dummy_progress(unsigned int step, char *s) {}

/* Called from arch/kernel/setup.c */
void c6x_board_setup_arch(void)
{   
	printk("Designed for the EVMC6678 board, Texas Instruments.\n");

	mach_progress      = dummy_progress;
	mach_print_value   = dummy_print_dummy;

	c6x_clk_init(evm_clks);

	mach_progress(1, "End of EVMC6678 specific initialization");
}

__init int evm_init(void)
{
	return 0;
}

arch_initcall(evm_init);
