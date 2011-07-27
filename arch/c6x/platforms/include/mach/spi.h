/*
 *  linux/arch/c6x/platforms/include/mach/spi.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2009, 2011 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __MACH_C6X_SPI_H
#define __MACH_C6X_SPI_H

enum {
	SPI_VERSION_1,  /* For DM355/DM365/DM6467/C667x */
	SPI_VERSION_2,  /* For DA8xx */
};

struct davinci_spi_platform_data {
	u8	version;
	u8	num_chipselect;
	u8	wdelay;
	u8	odd_parity;
	u8	parity_enable;
	u8	wait_enable;
	u8	timer_disable;
	u8	clk_internal;
	u8	cs_hold;
	u8	intr_level;
	u8	poll_mode;
	u8	use_dma;
	u8	c2tdelay;
	u8	t2cdelay;
};

/*
 * SPI base register addresses
 */
#ifdef CONFIG_TMS320C66X
#define SPI_REGISTER_BASE 0x20bf0000
#define SPI_REGISTER_END  0x20bf03ff
#else
#error "No machine SPI definitions"
#endif

#endif	/* __MACH_C6X_SPI_H */
