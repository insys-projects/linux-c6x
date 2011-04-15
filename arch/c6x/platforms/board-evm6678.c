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

#include <mach/board.h>

SOC_CLK_DEF(100000000); /* SYSCLK is a 100 MHz clock */

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
	.n_channel		= EDMA_NUM_DMACH,
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

#if defined(CONFIG_MTD_NAND_DAVINCI) || defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
#include <mach/emif.h>
#include <mach/nand.h>

static struct mtd_partition evm6678_nand_parts[] = {
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

static struct davinci_nand_pdata evmc6678_nand_data = {
	.mask_cle 		= 0x4000,
	.mask_ale 		= 0x2000,
	.parts			= evm6678_nand_parts,
	.nr_parts		= ARRAY_SIZE(evm6678_nand_parts),
	.ecc_mode		= NAND_ECC_HW,
	.ecc_bits               = 4,
	.options	        = NAND_USE_FLASH_BBT,
};

static struct resource evmc6678_nand_resources[] = {
	{
		.start		= RAM_EMIFA_CE2,
		.end		= RAM_EMIFA_CE2 + 0x3FFFFFF,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= EMIFA_BASE,
		.end		= EMIFA_BASE + 0xFF,
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

#define FPGA_MISC_REG_OFFSET 0x0c

#define FPGA_RD_CMD          (1 << 7)
#define FPGA_WR_CMD          (0 << 7)

static struct spi_device *fpga_spi_client;
static struct mutex	  fpga_lock;
static struct work_struct fpga_work;

static inline int fpga_spi_write_reg(u8 reg, u8 data)
{
	u16 buf = ((FPGA_WR_CMD | reg) << 8) | data;

	return spi_write(fpga_spi_client, (u8*) &buf, sizeof(buf));
}

static inline int fpga_spi_read_reg(u8 reg, u8 *p_data)
{
	u16 cmd = ((FPGA_RD_CMD | reg) << 8);
	u16 buf;
	int res;

	res = spi_write_then_read(fpga_spi_client,
				  (u8*) &cmd, sizeof(cmd),
				  (u8*) &buf, sizeof(buf));

	if (res < 0)
		return res;

	*p_data = (u8) buf & 0xff;

	return res;
}

static inline void evm_fpga_set(unsigned int state, u8 reg, u8 shift)
{
	u8  val = 0;
	int res;

	mutex_lock(&fpga_lock);

	res = fpga_spi_read_reg(reg, &val);
	if (res < 0) {
		mutex_unlock(&fpga_lock);
		return;
	}

	printk(KERN_INFO "FPGA reg 0x%x = 0x%x\n", reg, val);

	val &= ~(1 << shift);
	val |= state << shift;

	(void) fpga_spi_write_reg(reg, val);

	res = fpga_spi_read_reg(reg, &val);
	if (res < 0) {
		mutex_unlock(&fpga_lock);
		return;
	}

	printk(KERN_INFO "FPGA reg 0x%x after write = 0x%x\n", reg, val);

	mutex_unlock(&fpga_lock);
}
static void evm_fpga_work(struct work_struct *work)
{
#if 0
	/* Disable NOR write protect */
	evm_fpga_set(1, FPGA_MISC_REG_OFFSET, 4);

	/* Disable NAND write protect */
	evm_fpga_set(0, FPGA_MISC_REG_OFFSET, 2);

	/* Disable EEPROM write protect */
	evm_fpga_set(1, FPGA_MISC_REG_OFFSET, 5);
#endif
}

static int __devinit evm_fpga_probe(struct spi_device *spi)
{
	fpga_spi_client = spi;

	INIT_WORK(&fpga_work, evm_fpga_work);

	mutex_init(&fpga_lock);

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
	{
		.modalias	= "m25p80",
		.platform_data	= &n25q128,
		.max_speed_hz	= 25000000,
		.bus_num	= 0,
		.chip_select	= 0,
		.mode		= SPI_MODE_1,
		.bits_per_word  = 8,
	},
	{
		.modalias	= "FPGA",
		.platform_data	= NULL,
		.max_speed_hz	= 25000000,
		.bus_num	= 0,
		.chip_select	= 1,
		.mode		= SPI_MODE_0,
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
	.cs_hold	= 1,
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
                .irq      = IRQ_UART,
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
