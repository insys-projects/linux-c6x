/*
 *  drivers/mtd/nand/evm6488.c
 *
 *  NAND flash bit-bang driver for EVM6488 using GPIO
 *
 *  Copyright (C) 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@virtuallogix.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/gpio.h>

#undef NAND_DEBUG
#ifdef NAND_DEBUG
#define DPRINTK(ARGS...)                do { \
		                                printk(KERN_DEBUG "<%s>: ",__FUNCTION__);printk(ARGS);	\
	                                } while (0)
#else
#define DPRINTK( x... )
#endif

/* see datasheets (tR) */
#define NAND_BIG_DELAY_US		20
#define NAND_SMALL_DELAY_US		10
#define NAND_SMALL_SIZE                 0x02000000

#define USE_READY_BUSY_PIN

/* DSP GPIO pins used to drive NAND chip  */
#define NAND_CLE_GPIO_PIN 		(GPIO_PIN8)
#define NAND_ALE_GPIO_PIN 		(GPIO_PIN9)
#define NAND_NRE_GPIO_PIN 		(GPIO_PIN12)
#define NAND_NWE_GPIO_PIN 		(GPIO_PIN10)
#undef  NAND_NWP_GPIO_PIN               /* we do not use WP */
#define NAND_RNB_GPIO_PIN               (GPIO_PIN11)
#define NAND_NCE_GPIO_PIN               (GPIO_PIN13)

#define NAND_SETNCE()                   gpio_pin_clear(NAND_NCE_GPIO_PIN)   /* nCE low */
#define NAND_CLRNCE()                   gpio_pin_set(NAND_NCE_GPIO_PIN)     /* nCE high */

//#define GPIO_MASK                       0x00003FFF /* we do not use GPIO 14 and 15 pins */
#define GPIO_MASK                       0x00007FFF
#define GPIO_OUTPORTCONF                0x00000800
#define GPIO_INPORTCONF                 0x000008FF

#define GPIO_DATAMASK                   GPIO_INPORTCONF

/*
 * MTD structure for EVM6488
 */
static struct mtd_info  *evm6488_mtd = NULL;

#ifdef CONFIG_MTD_PARTITIONS
/*
 * Define static partitions for flash devices
 */
#if defined(CONFIG_ARCH_EVM6488)
static struct mtd_partition partition_info_evm6488[] = {
	{ name: "EVM6488 Nand Flash",
	  offset: 0,
	  size: 32*1024*1024 }
};
#elif defined(CONFIG_ARCH_BOARD_EVM6457) || defined(CONFIG_ARCH_BOARD_EVM6472)
static struct mtd_partition partition_info_evm6488[] = {
	{ name: "EVM NAND Flash",
	  offset: 0,
	  size: 128 * 1024 * 1024 }
};
#endif

#define NUM_PARTITIONS	1

extern int parse_cmdline_partitions(struct mtd_info *master,
				    struct mtd_partition **pparts,
				    const char *mtd_id);
#endif

#ifdef NAND_DEBUG
#define get_hrt() (get_cycles() & 0xffffffff)

static inline void nand_dump(void)
{
	u16 val = gpio_get_reg(GPIO_GPOUTDATA);
	char cle, nce, nwe, ale, nre, nwp;
	static unsigned long time_off = 0;
	unsigned long delta;

	if (val & (1 << NAND_CLE_GPIO_PIN))
		cle = '1';
	else
		cle = '0';

	if (val & (1 << NAND_NCE_GPIO_PIN))
		nce = '1';
	else
		nce = '0';

	if (val & (1 << NAND_ALE_GPIO_PIN))
		ale = '1';
	else
		ale = '0';

	if (val & (1 << NAND_NRE_GPIO_PIN))
		nre = '1';
	else
		nre = '0';

	if (val & (1 << NAND_NWE_GPIO_PIN))
		nwe = '1';
	else
		nwe = '0';

#ifdef NAND_NWP_GPIO_PIN
	if (val & (1 << NAND_NWP_GPIO_PIN))
		nwp = '1';
	else
		nwp = '0';
#else
	nwp = 'x';
#endif
	delta = get_hrt() - time_off;
	time_off = get_hrt();

	printk("NAND status (0x%x): ", val);
	printk("CLE %c nCE %c nWE %c ALE %c nRE %c nWP %c data 0x%x time = 0x%lx\n",
	       cle, nce, nwe, ale, nre, nwp, val & GPIO_DATAMASK, delta);
}
#else
#define nand_dump()
#endif

/**
 * Write a 8bit data through GPIO
 */
static inline void nand_write_data(u8 data)
{
        unsigned long flags;

	local_irq_save(flags);

	gpio_pin_clear(NAND_NWE_GPIO_PIN);

	ndelay(12);

	gpio_clear(GPIO_DATAMASK);
	gpio_set(data);

	ndelay(12);

	gpio_pin_set(NAND_NWE_GPIO_PIN);

	ndelay(12);

	local_irq_restore(flags);
}

/**
 * Read a 8bit data through GPIO
 */
static inline u8 nand_read_data(void)
{
        unsigned long flags;
	u8 val;

	local_irq_save(flags);

	gpio_direction_set(GPIO_INPORTCONF, GPIO_MASK);

	gpio_pin_clear(NAND_NRE_GPIO_PIN);

	ndelay(12);

	val = (u8) (gpio_input() & GPIO_DATAMASK);
	gpio_pin_set(NAND_NRE_GPIO_PIN);

	ndelay(12);

	gpio_direction_set(GPIO_OUTPORTCONF, GPIO_MASK);

	ndelay(12);

	local_irq_restore(flags);

	return val;
}

/**
 * nand_read_byte -  read one byte from the chip
 * @mtd:	MTD device structure
 *
 *  read function for 8bit buswith
 */
static u_char nand_read_byte(struct mtd_info *mtd)
{
	u_char ret = 0;

	ret = nand_read_data();

	return ret;
}

/**
 * nand_read_word -  read one word from the chip
 * @mtd:	MTD device structure
 *
 * read function for 16bit buswith without
 * endianess conversion
 */
static u16 nand_read_word(struct mtd_info *mtd)
{
	u16 ret = 0;
	u8  val1, val2;

	val1 = nand_read_data();
	val2 = nand_read_data();
	ret = (val1 << 16) | (val2 & 0xff);

	DPRINTK("val = 0x%x\n", ret);

	return ret;
}

/**
 * nand_write_buf -  write buffer to chip
 * @mtd:	MTD device structure
 * @buf:	data buffer
 * @len:	number of bytes to write
 *
 *  write function for 8bit buswith
 */
static void nand_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	int i;

	for (i = 0; i < len; i++)
		nand_write_data(buf[i]);
}

/**
 * nand_read_buf -  read chip data into buffer
 * @mtd:	MTD device structure
 * @buf:	buffer to store date
 * @len:	number of bytes to read
 *
 *  read function for 8bit buswith
 */
static void nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	int i;

	for (i = 0; i < len; i++)
		buf[i] = nand_read_data();
}

/**
 * nand_verify_buf -  Verify chip data against buffer
 * @mtd:	MTD device structure
 * @buf:	buffer containing the data to compare
 * @len:	number of bytes to compare
 *
 *  verify function for 8bit buswith
 */
static int nand_verify_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	int i;

	for (i = 0; i < len; i++)
	        if (buf[i] != nand_read_data())
		        return -EFAULT;

	return 0;
}

/*
 *	hardware specific access to control-lines
 */
static void evm6488_hwcontrol(struct mtd_info *mtdinfo, int cmd, unsigned int ctrl)
{
	if (ctrl & NAND_CTRL_CHANGE) {
		if (ctrl & NAND_NCE)
			gpio_pin_clear(NAND_NCE_GPIO_PIN);
		else
			gpio_pin_set(NAND_NCE_GPIO_PIN);

	        if (ctrl & NAND_CLE)
			gpio_pin_set(NAND_CLE_GPIO_PIN);
		else
			gpio_pin_clear(NAND_CLE_GPIO_PIN);

	        if (ctrl & NAND_ALE)
			gpio_pin_set(NAND_ALE_GPIO_PIN);
		else
			gpio_pin_clear(NAND_ALE_GPIO_PIN);
	}

	if (cmd != NAND_CMD_NONE)
		nand_write_data(cmd);
}

#ifdef USE_READY_BUSY_PIN
/*
 * Read device ready pin
 */
static int evm6488_device_ready(struct mtd_info *minfo)
{
	if (gpio_input() & (1 << NAND_RNB_GPIO_PIN))
		return 1;
	return 0;
}

#endif

#ifdef CONFIG_MTD_PARTITIONS
const char *part_probes[] = { "cmdlinepart", NULL };
#endif

/*
 * Main initialization routine
 */
static int __init evm6488_nand_init (void)
{
	struct nand_chip     *this;
	const char           *part_type = 0;
	int                   mtd_parts_nb = 0;
	struct mtd_partition *mtd_parts = 0;
        unsigned long         flags;

	DPRINTK("starting init\n");

	/* Allocate memory for MTD device structure and private data */
	evm6488_mtd = kmalloc(sizeof(struct mtd_info) +
			      sizeof(struct nand_chip), GFP_KERNEL);
	if (!evm6488_mtd) {
		printk("Unable to allocate EVM6488 NAND MTD device structure.\n");
		return -ENOMEM;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *) (&evm6488_mtd[1]);

	/* Initialize structures */
	memset((char *) evm6488_mtd, 0, sizeof(struct mtd_info));
	memset((char *) this, 0, sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	evm6488_mtd->priv = this;

	printk("EVM6488 NAND Controller\n");

	/* Init GPIO pins */
	local_irq_save(flags);

	gpio_direction_set(GPIO_OUTPORTCONF, GPIO_MASK);
	gpio_pin_set(GPIO_PIN14);
	gpio_pin_clear(NAND_CLE_GPIO_PIN);   /* CLE low */
	NAND_CLRNCE();                       /* nCE high */
	gpio_pin_set(NAND_NWE_GPIO_PIN);     /* nWE high */
	gpio_pin_clear(NAND_ALE_GPIO_PIN);   /* ALE low */
	gpio_pin_set(NAND_NRE_GPIO_PIN);     /* nRE high */

	nand_dump();

	local_irq_restore(flags);

#ifdef NAND_NWP_GPIO_PIN
	gpio_pin_set(NAND_NWP_GPIO_PIN);     /* nWP high */
#endif
	nand_dump();

	/* No mapped I/O, use GPIO instead  */
	this->IO_ADDR_R  = NULL;
	this->IO_ADDR_W  = NULL;

	/* Insert callbacks */
	this->cmd_ctrl   = evm6488_hwcontrol;
#ifdef USE_READY_BUSY_PIN
	this->dev_ready  = evm6488_device_ready;
#endif
	this->chip_delay = NAND_BIG_DELAY_US;
	this->ecc.mode   = NAND_ECC_SOFT;	/* ECC mode */
	this->read_byte  = nand_read_byte;
//	this->write_byte = nand_write_byte;
//	this->write_word = nand_write_word;
	this->read_word  = nand_read_word;
	this->write_buf  = nand_write_buf;
	this->read_buf   = nand_read_buf;
	this->verify_buf = nand_verify_buf;
	this->options   |= NAND_COPYBACK;

	DPRINTK("scanning\n");

	/* Scan to find existence of the device (it could not be mounted) */
	if (nand_scan(evm6488_mtd, 1)) {
		kfree(evm6488_mtd);
		return -ENXIO;
	}

	evm6488_mtd->owner = THIS_MODULE;

	DPRINTK("scanning finished\n");

#ifndef USE_READY_BUSY_PIN
	/* Adjust delay if necessary */
	if (evm6488_mtd->size == NAND_SMALL_SIZE)
		this->chip_delay = NAND_SMALL_DELAY_US;
#endif

#ifdef CONFIG_MTD_PARTITIONS
	evm6488_mtd->name = "EVM-NAND";
	mtd_parts_nb = parse_mtd_partitions(evm6488_mtd, part_probes, &mtd_parts, 0);
	if (mtd_parts_nb > 0)
		part_type = "command line";
	else
		mtd_parts_nb = 0;

	if (mtd_parts_nb == 0)
	{
		mtd_parts    = partition_info_evm6488;
		mtd_parts_nb = NUM_PARTITIONS;
		part_type    = "static";
	}

	/* Register the partitions */
	printk(KERN_NOTICE "Using %s partition definition\n", part_type);
	if (add_mtd_partitions(evm6488_mtd, mtd_parts, mtd_parts_nb)) {
		printk(KERN_ERR "can't register partitions\n");
	}
#endif
	return 0;
}
module_init(evm6488_nand_init);

/*
 * Clean up routine
 */
static void __exit evm6488_nand_cleanup (void)
{
	/* Release resources, unregister device(s) */
	nand_release(evm6488_mtd);

	/* Free the MTD device structure */
	kfree (evm6488_mtd);
}
module_exit(evm6488_nand_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aurelien Jacquiot <aurelien.jacquiot@vlx.com>");
MODULE_DESCRIPTION("MTD map driver for EVM6488 board");
