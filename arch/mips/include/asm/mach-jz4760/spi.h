/*
 * linux/arch/mips/include/asm/mach-jz4760b/spi.h
 *
 * SSI controller for SPI protocol,use FIFO and DMA;
 *
 * Copyright (c) 2010 Ingenic Semiconductor Inc.
 * Author: Shumb <sbhuang@ingenic.cn>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 */

#ifndef __I_SPI_H__
#define __I_SPI_H__

/* the max number of spi devices */
#define MAX_SPI_DEVICES				10
#define MAX_SPI_HOST				2

#define JZ_SPI_ID_INVALID(ssi_id) ( ((ssi_id) < 0) || ((ssi_id) > (MAX_SPI_HOST - 1)) )

#define MAX_SPI_CHIPSELECT_NUM 		MAX_GPIO_NUM

#define PIN_SSI_CE0 	0
#define PIN_SSI_CE1 	1

struct jz47xx_spi_info {
	u8	chnl;								/* the chanel of SSI controller */
	u16	bus_num;							/* spi_master.bus_num */
	unsigned is_pllclk:1;					/* source clock: 1---pllclk;0---exclk */
	unsigned long		 board_size;		/* spi_master.num_chipselect */
	struct spi_board_info	*board_info; 	/* link to spi devices info */
	void (*set_cs)(struct jz47xx_spi_info *spi, u8 cs,unsigned int pol); /* be defined by spi devices driver user */
	void (*pins_config)(void);				/* configure spi function pins (CLK,DR,RT) by user if need. */
	u32	 num_chipselect;
};

/* Chipselect "set_cs" function could be defined by user. Example as the follow ... */
/*
static void spi_gpio_cs(struct jz47xx_spi_info *spi, int cs, int pol);
{
	int pinval;

	switch(cs){
	case 0:
		pinval = 32*1+31;
		break;
	case 1:
		pinval = 32*1+30;
		break;
	case 2:
		pinval = 32*1+29;
		break;
	default:
		pinval = 32*1+28;
		break;	
	}
	__gpio_as_output(pinval);				
	switch (pol) {
	case BITBANG_CS_ACTIVE:
		__gpio_set_pin(pinval);
		break;
	case BITBANG_CS_INACTIVE:
		__gpio_clear_pin(pinval);
		break;
	}

}*/

#endif
