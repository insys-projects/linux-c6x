#ifndef __LINUX_MTD_NAND_EVM6488_H
#define __LINUX_MTD_NAND_EVM6488_H

#include <linux/mtd/nand.h>

struct gpio_nand_platdata {
	void	(*adjust_parts)(struct gpio_nand_platdata *, size_t);
	struct mtd_partition *parts;
	unsigned int num_parts;
	unsigned int options;
	int	chip_delay;
};

#endif
