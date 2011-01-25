/*
 *  drivers/mtd/nand/gpio-c6x.c
 *
 *  GPIO based NAND driver for C6x SoC
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Mark Salter <msalter@redhat.com>
 *  Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  Based on gpio.c:
 *
 * Updated, and converted to generic GPIO based driver by Russell King.
 *
 * Written by Ben Dooks <ben@simtec.co.uk>
 *   Based on 2.4 version by Mark Whittaker
 *
 * © 2004 Simtec Electronics
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand-gpio-c6x.h>

#define NAND_MTD_NAME "gpio-nand-c6x"

struct gpiomtd {
	struct mtd_info		mtd_info;
	struct nand_chip	nand_chip;
	struct gpio_nand_platdata plat;
	struct gpio_controller *__iomem g;
};

#define gpio_nand_getpriv(x) container_of(x, struct gpiomtd, mtd_info)

/*
 * The generic GPIO interface doesn't provide an efficient way to
 * read/write groups of GPIO lines as is needed for NAND data. The
 * routines to do this are very specific to the C6X GPIO controller...
 */
static inline u_char gpio_nand_read_data(struct gpiomtd *gpiomtd)
{
	struct gpio_controller *__iomem g = gpiomtd->g;
	return __raw_readl(&g->in_data);
}

static inline void gpio_nand_write_data(struct gpiomtd *gpiomtd, u_char val)
{
	struct gpio_controller *__iomem g = gpiomtd->g;
	__raw_writel(GPIO_NAND_DATA_MASK, &g->clr_data);
	__raw_writel(val, &g->set_data);
}

static inline void gpio_nand_direction_out(struct gpiomtd *gpiomtd)
{
	struct gpio_controller *__iomem g = gpiomtd->g;

	__dint();
	__raw_writel(__raw_readl(&g->dir) & ~GPIO_NAND_DATA_MASK, &g->dir);
	__rint();
}

static inline void gpio_nand_direction_in(struct gpiomtd *gpiomtd)
{
	struct gpio_controller *__iomem g = gpiomtd->g;

	__dint();
	__raw_writel(__raw_readl(&g->dir) | GPIO_NAND_DATA_MASK, &g->dir);
	__rint();
}

static inline u_char gpio_nand_read(struct gpiomtd *gpiomtd)
{
	u_char val;

	gpio_set_value(GPIO_NAND_NRE, 0);
	xdelay();
	val = gpio_nand_read_data(gpiomtd);
	gpio_set_value(GPIO_NAND_NRE, 1);
	xdelay();

	return val;
}

static inline void gpio_nand_write(struct gpiomtd *gpiomtd, u_char val)
{

	gpio_set_value(GPIO_NAND_NWE, 0);
	gpio_nand_write_data(gpiomtd, val);
	xdelay();
	gpio_set_value(GPIO_NAND_NWE, 1);
	xdelay();
}

static void gpio_nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct gpiomtd *gpiomtd = gpio_nand_getpriv(mtd);

	if (ctrl & NAND_CTRL_CHANGE) {
		gpio_set_value(GPIO_NAND_NCE, !(ctrl & NAND_NCE));
		gpio_set_value(GPIO_NAND_CLE, !!(ctrl & NAND_CLE));
		gpio_set_value(GPIO_NAND_ALE, !!(ctrl & NAND_ALE));
	}
	if (cmd == NAND_CMD_NONE)
		return;

	gpio_nand_write(gpiomtd, cmd);
}

static void gpio_nand_writebuf(struct mtd_info *mtd, const u_char *buf, int len)
{
	struct gpiomtd *gpiomtd = gpio_nand_getpriv(mtd);

	while (len-- > 0) {
		gpio_set_value(GPIO_NAND_NWE, 0);
		gpio_nand_write_data(gpiomtd, *buf++);
		xdelay();
		gpio_set_value(GPIO_NAND_NWE, 1);
		xdelay();
	}
}

static void gpio_nand_readbuf(struct mtd_info *mtd, u_char *buf, int len)
{
	struct gpiomtd *gpiomtd = gpio_nand_getpriv(mtd);

	gpio_nand_direction_in(gpiomtd);

	while (len-- > 0)
		*buf++ = gpio_nand_read(gpiomtd);

	gpio_nand_direction_out(gpiomtd);
}

static uint8_t gpio_nand_read_byte(struct mtd_info *mtd)
{
	uint8_t val;

	gpio_nand_readbuf(mtd, &val, 1);

	return val;
}

static int gpio_nand_verifybuf(struct mtd_info *mtd, const u_char *buf, int len)
{
	struct gpiomtd *gpiomtd = gpio_nand_getpriv(mtd);
	unsigned char read, *p = (unsigned char *) buf;
	int i, err = 0;

	gpio_nand_direction_in(gpiomtd);

	for (i = 0; i < len; i++) {
		read = gpio_nand_read(gpiomtd);
		if (read != p[i]) {
			pr_debug("%s: err at %d (read %04x vs %04x)\n",
			       __func__, i, read, p[i]);
			err = -EFAULT;
		}
	}

	gpio_nand_direction_out(gpiomtd);

	return err;
}

static int gpio_nand_devready(struct mtd_info *mtd)
{
	return gpio_get_value(GPIO_NAND_RDY);
}

static int __devexit gpio_nand_remove(struct platform_device *dev)
{
	struct gpiomtd *gpiomtd = platform_get_drvdata(dev);
	int i;

	nand_release(&gpiomtd->mtd_info);

	gpio_set_value(GPIO_NAND_NCE, 1);

	gpio_free(GPIO_NAND_CLE);
	gpio_free(GPIO_NAND_ALE);
	gpio_free(GPIO_NAND_NCE);
#ifdef GPIO_NAND_USE_READY_BUSY_PIN
	gpio_free(GPIO_NAND_RDY);
#endif
	gpio_free(GPIO_NAND_NWE);
	gpio_free(GPIO_NAND_NRE);
#ifdef GPIO_NAND_USE_WRITE_PROTECT
	gpio_free(GPIO_NAND_NWP);
#endif
	/* NAND data lines */
	for (i = 0; i < 8; i++)
		gpio_free(i);

	kfree(gpiomtd);

	return 0;
}

#ifdef CONFIG_MTD_PARTITIONS
const char *part_probes[] = { "cmdlinepart", "RedBoot", NULL };
#endif

static int __devinit gpio_nand_probe(struct platform_device *dev)
{
	struct gpiomtd       *gpiomtd;
	struct nand_chip     *this;
	const char           *part_type    = 0;
	struct mtd_partition *mtd_parts    = 0;
	int                   mtd_parts_nb = 0;
	int                   i, ret;

	if (!dev->dev.platform_data)
		return -EINVAL;

	gpiomtd = kzalloc(sizeof(*gpiomtd), GFP_KERNEL);
	if (gpiomtd == NULL) {
		dev_err(&dev->dev, "failed to create NAND MTD\n");
		return -ENOMEM;
	}

	memcpy(&gpiomtd->plat, dev->dev.platform_data, sizeof(gpiomtd->plat));
	this = &gpiomtd->nand_chip;

	gpiomtd->g = __gpio_to_controller(0);

#ifdef GPIO_NAND_USE_WRITE_PROTECT
	ret = gpio_request(GPIO_NAND_NWP, "NAND MISC");
	if (ret)
		goto err_nwp;
	gpio_direction_output(GPIO_NAND_NWP, 1);
#endif
	ret = gpio_request(GPIO_NAND_NCE, "NAND NCE");
	if (ret)
		goto err_nce;
	gpio_direction_output(GPIO_NAND_NCE, 1);

	ret = gpio_request(GPIO_NAND_ALE, "NAND ALE");
	if (ret)
		goto err_ale;
	gpio_direction_output(GPIO_NAND_ALE, 0);

	ret = gpio_request(GPIO_NAND_CLE, "NAND CLE");
	if (ret)
		goto err_cle;
	gpio_direction_output(GPIO_NAND_CLE, 0);

#ifdef GPIO_NAND_USE_READY_BUSY_PIN
	ret = gpio_request(GPIO_NAND_RDY, "NAND RDY");
	if (ret)
		goto err_rdy;
	gpio_direction_input(GPIO_NAND_RDY);
#endif
	ret = gpio_request(GPIO_NAND_NWE, "NAND NWE");
	if (ret)
		goto err_nwe;
	gpio_direction_output(GPIO_NAND_NWE, 1);

	ret = gpio_request(GPIO_NAND_NRE, "NAND NRE");
	if (ret)
		goto err_nre;
	gpio_direction_output(GPIO_NAND_NRE, 1);

	for (i = 0; i < 8; i++) {
		ret = gpio_request(i, "NAND Data");
		if (ret) {
			while (--i > 0)
				gpio_free(i);
			goto err_wp;
		}
	}
	for (i = 0; i < 8; i++)
		gpio_direction_output(i, 0);

	this->ecc.mode   = NAND_ECC_SOFT;
	this->options    = gpiomtd->plat.options;
	this->chip_delay = gpiomtd->plat.chip_delay;

	/* install our routines */
	this->cmd_ctrl   = gpio_nand_cmd_ctrl;
#ifdef GPIO_NAND_USE_READY_BUSY_PIN
	this->dev_ready  = gpio_nand_devready;
#endif
	this->read_buf   = gpio_nand_readbuf;
	this->read_byte  = gpio_nand_read_byte;
	this->write_buf  = gpio_nand_writebuf;
	this->verify_buf = gpio_nand_verifybuf;

	/* set the mtd private data for the nand driver */
	gpiomtd->mtd_info.priv  = this;
	gpiomtd->mtd_info.owner = THIS_MODULE;
	gpiomtd->mtd_info.name  = NAND_MTD_NAME;

	if (nand_scan(&gpiomtd->mtd_info, 1)) {
		dev_err(&dev->dev, "no nand chips found?\n");
		ret = -ENXIO;
		goto err_data;
	}

	/*
	 * Partition selection stuff.
	 */
#ifdef CONFIG_MTD_PARTITIONS
	mtd_parts_nb = parse_mtd_partitions(&gpiomtd->mtd_info, part_probes, &mtd_parts, 0);
	if (mtd_parts_nb > 0) {
		part_type = "dynamic";
	}
#endif	
	if (mtd_parts_nb == 0) {
		mtd_parts    = gpiomtd->plat.parts;
		mtd_parts_nb = gpiomtd->plat.num_parts;

		if (gpiomtd->plat.adjust_parts)
			gpiomtd->plat.adjust_parts(&gpiomtd->plat,
						   gpiomtd->mtd_info.size);
		part_type = "static";
	}

	if (mtd_parts_nb == 0) {
		printk(KERN_NOTICE "SA1100 flash: no partition info "
		       "available, registering whole flash\n");
		add_mtd_device(&gpiomtd->mtd_info);
	} else {
		printk(KERN_NOTICE "Using %s partition definition\n", part_type);
		if (add_mtd_partitions(&gpiomtd->mtd_info, mtd_parts, mtd_parts_nb)) {
			printk(KERN_ERR "can't register partitions\n");
		}
	}

	platform_set_drvdata(dev, gpiomtd);

	return 0;

err_data:
	for (i = 0; i < 8; i++)
		gpio_free(i);
err_wp:
	gpio_free(GPIO_NAND_NRE);
err_nre:
	gpio_free(GPIO_NAND_NWE);
err_nwe:
#ifdef GPIO_NAND_USE_READY_BUSY_PIN
	gpio_free(GPIO_NAND_RDY);
err_rdy:
#endif
	gpio_free(GPIO_NAND_CLE);
err_cle:
	gpio_free(GPIO_NAND_ALE);
err_ale:
	gpio_free(GPIO_NAND_NCE);
err_nce:
#ifdef GPIO_NAND_USE_WRITE_PROTECT
	gpio_free(GPIO_NAND_NWP);
err_nwp:
#endif
	kfree(gpiomtd);
	return ret;
}

static struct platform_driver gpio_nand_driver = {
	.probe		= gpio_nand_probe,
	.remove		= gpio_nand_remove,
	.driver		= {
		.name	= NAND_MTD_NAME,
	},
};

static int __init gpio_nand_init(void)
{
	printk(KERN_INFO "GPIO NAND driver for C6x SoC boards\n");

	return platform_driver_register(&gpio_nand_driver);
}

static void __exit gpio_nand_exit(void)
{
	platform_driver_unregister(&gpio_nand_driver);
}

module_init(gpio_nand_init);
module_exit(gpio_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mark Salter <msalter@redhat.com>");
MODULE_AUTHOR("Aurelien Jacquiot <a-jacquiot@ti.com>");
MODULE_DESCRIPTION("GPIO NAND Driver for C6x SoC boards");
