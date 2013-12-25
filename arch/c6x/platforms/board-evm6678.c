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

/*
 * LEDs management
 */
#define PIN_A       0
#define PIN_B       1
#define PIN_C       2
#define PIN_D       3
#define PIN_E       4
#define PIN_F       7
#define PIN_G       8
#define PIN_DOT     9

#define GPIO_OUT    0
#define GPIO_IN     1
#define GPIO_BASE_REG		0x02320000
#define GPIO_SIZE		0x100
#define GPIO_DIR_REG		0x10
#define GPIO_SET_DATA_REG	0x18
#define GPIO_CLEAR_DATA_REG	0x1C

volatile static u32* gpio0 = 0;

static void hwGpioSetDirection( u32 uiNumber, int direction )
{
    if ( direction == GPIO_OUT )
        gpio0[GPIO_DIR_REG/4] &= ~(1 << uiNumber);  // Set to OUTPUT
    else
        gpio0[GPIO_DIR_REG/4] |= ~(1 << uiNumber);  // Set to INPUT
}

static void hwGpioSetOutput( u32 uiNumber)
{
    gpio0[GPIO_SET_DATA_REG/4] = ( 1 << (uiNumber % 32) );  // Set to 1
}

static void hwGpioClearOutput( u32 uiNumber)
{
    gpio0[GPIO_CLEAR_DATA_REG/4] = ( 1 << (uiNumber % 32) );   // Clear to 0
}

static void LED_init(void)
{
    hwGpioSetDirection(PIN_A, GPIO_OUT);
    hwGpioSetDirection(PIN_B, GPIO_OUT);
    hwGpioSetDirection(PIN_C, GPIO_OUT);
    hwGpioSetDirection(PIN_D, GPIO_OUT);
    hwGpioSetDirection(PIN_E, GPIO_OUT);
    hwGpioSetDirection(PIN_F, GPIO_OUT);
    hwGpioSetDirection(PIN_G, GPIO_OUT);
    hwGpioSetDirection(PIN_DOT, GPIO_OUT);
}

static void LED_off(void)
{
    hwGpioSetOutput(PIN_A);
    hwGpioSetOutput(PIN_B);
    hwGpioSetOutput(PIN_C);
    hwGpioSetOutput(PIN_D);
    hwGpioSetOutput(PIN_E);
    hwGpioSetOutput(PIN_F);
    hwGpioSetOutput(PIN_G);
    hwGpioSetOutput(PIN_DOT);
}

static void LED_on(void)
{
    hwGpioClearOutput(PIN_A);
    hwGpioClearOutput(PIN_B);
    hwGpioClearOutput(PIN_C);
    hwGpioClearOutput(PIN_D);
    hwGpioClearOutput(PIN_E);
    hwGpioClearOutput(PIN_F);
    hwGpioClearOutput(PIN_G);
    hwGpioClearOutput(PIN_DOT);
}

static void LED_smart(int symbol)
{
    unsigned char   code = 0;

    switch(symbol) {
    case '0':	code = 0x3F; break;
    case '1':	code = 0x06; break;
    case '2':	code = 0x5B; break;
    case '3':	code = 0x4F; break;
    case '4':	code = 0x66; break;
    case '5':	code = 0x6D; break;
    case '6':	code = 0x7D; break;
    case '7':	code = 0x07; break;
    case '8':	code = 0x7F; break;
    case '9':	code = 0x6F; break;
    case 'A':	code = 0x77; break;
    case 'b':	code = 0x7C; break;
    case 'c':	code = 0x58; break;
    case 'C':	code = 0x39; break;
    case 'd':	code = 0x5E; break;
    case 'E':	code = 0x79; break;
    case 'F':	code = 0x71; break;
    case 'h':	code = 0x74; break;
    case 'H':	code = 0x76; break;
    case 'L':	code = 0x38; break;
    case 'o':	code = 0x5C; break;
    case 'P':	code = 0x73; break;
    case 'u':	code = 0x1C; break;
    case 'U':	code = 0x3E; break;
    case 'Y':	code = 0x6E; break;
    case 'S':	code = 0xED; break;
    }

    LED_off();

    if((code>>0)&1) hwGpioClearOutput(PIN_A);   // LED ON
    if((code>>1)&1) hwGpioClearOutput(PIN_B);   // LED ON
    if((code>>2)&1) hwGpioClearOutput(PIN_C);   // LED ON
    if((code>>3)&1) hwGpioClearOutput(PIN_D);   // LED ON
    if((code>>4)&1) hwGpioClearOutput(PIN_E);   // LED ON
    if((code>>5)&1) hwGpioClearOutput(PIN_F);   // LED ON
    if((code>>6)&1) hwGpioClearOutput(PIN_G);   // LED ON
    if((code>>7)&1) hwGpioClearOutput(PIN_DOT); // LED ON
}

#include <linux/timer.h>
static struct timer_list leds_timer;

static void board_leds_timer(unsigned long dummy)
{
    u32 *cnt = (u32*)dummy;

    if(cnt[0] == 0) {
        hwGpioSetOutput(PIN_DOT);
        cnt[0] = 1;
    } else {
        hwGpioClearOutput(PIN_DOT);
        cnt[0] = 0;
    }

    mod_timer(&leds_timer, jiffies + msecs_to_jiffies(250));
}

static int __init board_init_leds(void)
{
    u32 *counter = 0;

    gpio0 = (u32*)ioremap_nocache(GPIO_BASE_REG, GPIO_SIZE);
    if(!gpio0) {
       printk("<0>%s(): Error remap GPIO region\n", __FUNCTION__);
       return -1;
    }

    LED_init();
    LED_off();
    LED_smart('L');

    counter = (u32*)kmalloc(4,GFP_ATOMIC);

    init_timer(&leds_timer);
    leds_timer.function = board_leds_timer;
    leds_timer.data = counter;
    mod_timer(&leds_timer, jiffies + msecs_to_jiffies(250));

    return 0;
}
device_initcall(board_init_leds);

/*
 * Idle LED blink: LEDs are async due to SPI thus this feature cannot be impemented
 */
void c6x_arch_idle_led(int state) { }

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/spi/eeprom.h>
#include <mach/spi.h>

/*
 * SPI NOR Flash support
 */
static struct mtd_partition n25q128_evm_partitions[] = {
    {
        .name           = "kernel",
        .offset         = 0,
        .size           = 0xA00000,
        .mask_flags     = 0,
    },
    {
        .name		= "filesystem",
        .offset		= MTDPART_OFS_APPEND,
        .size		= MTDPART_SIZ_FULL,
        .mask_flags	= 0,
    }
};

static const struct flash_platform_data n25q128 = {
    .type       = "n25q128",
	.name		= "spi_flash",
	.parts		= n25q128_evm_partitions,
	.nr_parts	= ARRAY_SIZE(n25q128_evm_partitions),
};

/*
 * SPI
 */
static struct spi_board_info evm_spi_info[] __initconst = {
	/*
	 * SPI access to the FPGA for LED blinking is today incompatible 
	 * with the NOR flash access. So set them exclusive.
	 */
	{
        .modalias = "m25p80",
        .platform_data = &n25q128,
        .max_speed_hz = 25000000,
		.bus_num	= 0,
        .chip_select = 0,
        .mode = SPI_MODE_0,
        .bits_per_word = 8,
	},
#if 0
	{
		.modalias	= "FPGA",
		.platform_data	= NULL,
		.max_speed_hz	= 25000000,
		.bus_num	= 0,
		.chip_select	= 1,
		.mode		= SPI_MODE_1,
		.bits_per_word  = 16,
	},
#endif
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
    .poll_mode	= 1, /* 0 -> interrupt mode, 1-> polling mode */
	.c2tdelay	= 0,
	.t2cdelay	= 0,
    .use_dma    = 0, /* do not use EDMA */
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
    int res;

    res = platform_device_register(&evm_spi0_device);
    if(res < 0) {
        printk("<0>%s(): Error in platform_device_register()\n", __FUNCTION__);
        return res;
    }

    res = spi_register_board_info(evm_spi_info, ARRAY_SIZE(evm_spi_info));
    if(res < 0) {
        printk("<0>%s(): Error in spi_register_board_info()\n", __FUNCTION__);
    }

    printk("<0>%s() - SUCCESS\n", __FUNCTION__);

    return res;
}
core_initcall(board_setup_spi);

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
    { I2C_BOARD_INFO("tca9548apw", 0x70), },
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
    /* sRIO config 2: MPY = 8x, div rate = half, link rate = 5 Gbps, mode 1x */
	{ 0x0c053860, /* VBUS freq 313 - 626, promiscuous */
      0x0281,
      0x0010,
	  { 0x00440495, 0x00440495, 0x00440495, 0x00440495 },
	  { 0x00180795, 0x00180795, 0x00180795, 0x00180795 },
      { 0x00000000, 0x00000000, 0x00000000, 0x00000000 } },
    /* sRIO config 3: MPY = 8x, div rate = half, link rate = 5 Gbps, mode 4x */
	{ 0x0c053860, /* VBUS freq 313 - 626, promiscuous */
      0x0281,
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

static void dummy_print_dummy(char *s, unsigned long hex) { printk("<0>c6x: %s 0x%lx\n", s, hex); }
static void dummy_progress(unsigned int step, char *s) { printk("<0>c6x: %s\n", s); }

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
