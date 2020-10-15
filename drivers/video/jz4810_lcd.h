/*
 * linux/drivers/video/jz4810_lcd.h -- Ingenic Jz4810 On-Chip LCD frame buffer device
 *
 * Copyright (C) 2005-2008, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __JZ4810_LCD_H__
#define __JZ4810_LCD_H__

//#include <asm/io.h>


#define NR_PALETTE	256
#define PALETTE_SIZE	(NR_PALETTE*2)

struct reg{
	unsigned long da0;
	unsigned long da1;
}da_reg;

/* use new descriptor(8 words) */
struct jz4810_lcd_dma_desc {
	unsigned int next_desc; 	/* LCDDAx */
	unsigned int databuf;   	/* LCDSAx */
	unsigned int frame_id;  	/* LCDFIDx */
	unsigned int cmd; 		/* LCDCMDx */
	unsigned int offsize;       	/* Stride Offsize(in word) */
	unsigned int page_width; 	/* Stride Pagewidth(in word) */
	unsigned int cmd_num; 		/* Command Number(for SLCD) */
	unsigned int desc_size; 	/* Foreground Size */
};

struct jz4810lcd_panel_t {
	unsigned int cfg;	/* panel mode and pin usage etc. */
	unsigned int slcd_cfg;	/* Smart lcd configurations */
	unsigned int ctrl;	/* lcd controll register */
	unsigned int w;		/* Panel Width(in pixel) */
	unsigned int h;		/* Panel Height(in line) */
	unsigned int fclk;	/* frame clk */
	unsigned int hsw;	/* hsync width, in pclk */
	unsigned int vsw;	/* vsync width, in line count */
	unsigned int elw;	/* end of line, in pclk */
	unsigned int blw;	/* begin of line, in pclk */
	unsigned int efw;	/* end of frame, in line count */
	unsigned int bfw;	/* begin of frame, in line count */
};


struct jz4810lcd_fg_t {
	int bpp;	/* foreground bpp */
	int x;		/* foreground start position x */
	int y;		/* foreground start position y */
	int w;		/* foreground width */
	int h;		/* foreground height */
};

struct jz4810lcd_osd_t {
	unsigned int osd_cfg;	/* OSDEN, ALHPAEN, F0EN, F1EN, etc */
	unsigned int osd_ctrl;	/* IPUEN, OSDBPP, etc */
	unsigned int rgb_ctrl;	/* RGB Dummy, RGB sequence, RGB to YUV */
	unsigned int bgcolor;	/* background color(RGB888) */
	unsigned int colorkey0;	/* foreground0's Colorkey enable, Colorkey value */
	unsigned int colorkey1; /* foreground1's Colorkey enable, Colorkey value */
	unsigned int alpha;	/* ALPHAEN, alpha value */
	unsigned int ipu_restart; /* IPU Restart enable, ipu restart interval time */

#define FG_NOCHANGE 		0x0000
#define FG0_CHANGE_SIZE 	0x0001
#define FG0_CHANGE_POSITION 	0x0010
#define FG0_CHANGE_BUF  	0x0100
#define FG0_CHANGE_PALETTE 	0x1000
#define FG1_CHANGE_SIZE 	0x0002
#define FG1_CHANGE_POSITION 	0x0020
#define FG1_CHANGE_BUF  	0x0200
#define FG_CHANGE_ALL 		( FG0_CHANGE_SIZE | FG0_CHANGE_POSITION |\
				  FG1_CHANGE_SIZE | FG1_CHANGE_POSITION )
	int fg_change;
	struct jz4810lcd_fg_t fg0;	/* foreground 0 */
	struct jz4810lcd_fg_t fg0p2;	/* foreground 0p2 */
	struct jz4810lcd_fg_t fg1;	/* foreground 1 */
};

struct jz4810lcd_info {
	struct jz4810lcd_panel_t panel;
	struct jz4810lcd_osd_t osd;
	unsigned long frame0;
	unsigned long frame1;
	unsigned long frame2_0;
	unsigned long frame2_1;
	int without_alpha;
	int is0_compressed;
	int is1_compressed;
	int is0_disPart;
	int is1_disPart;
	int test_part2;
	unsigned long pdma00;
	unsigned long pdma10;
};

/* Jz LCDFB supported I/O controls. */
#define FBIOSETBACKLIGHT	0x4688 /* set back light level */
#define FBIODISPON		0x4689 /* display on */
#define FBIODISPOFF		0x468a /* display off */
#define FBIORESET		0x468b /* lcd reset */
#define FBIOPRINT_REG		0x468c /* print lcd registers(debug) */
#define FBIOROTATE		0x46a0 /* rotated fb */
#define FBIOGETBUFADDRS		0x46a1 /* get buffers addresses */
#define FBIO_GET_MODE		0x46a2 /* get lcd info */
#define FBIO_SET_MODE		0x46a3 /* set osd mode */
#define FBIO_DEEP_SET_MODE	0x46a4 /* set panel and osd mode */
#define FBIO_MODE_SWITCH	0x46a5 /* switch mode between LCD and TVE */
#define FBIO_GET_TVE_MODE	0x46a6 /* get tve info */
#define FBIO_SET_TVE_MODE	0x46a7 /* set tve mode */

/*
 * LCD panel specific definition
 */
/* AUO */
#if defined(CONFIG_JZ4810_LCD_AUO_A043FL01V2)
#if defined(CONFIG_JZ4810_F4810) 
	#define SPEN		(32*1+29)       /*LCD_CS*/
	#define SPCK		(32*1+28)       /*LCD_SCL*/
	#define SPDA		(32*1+21)       /*LCD_SDA*/
        #define LCD_RET 	(32*3+9)       /*GPD9 LCD_DISP_N use for lcd reset*/
#else
#error "driver/video/Jzlcd.h, please define SPI pins on your board."
#endif

#define __spi_write_reg(reg, val)		\
	do {					\
		unsigned char no;		\
		unsigned short value;		\
		unsigned char a=0;		\
		unsigned char b=0;		\
		__gpio_as_output(SPEN); /* use SPDA */	\
		__gpio_as_output(SPCK); /* use SPCK */	\
		__gpio_as_output(SPDA); /* use SPDA */	\
		a=reg;				\
		b=val;				\
		__gpio_set_pin(SPEN);		\
		__gpio_clear_pin(SPCK);		\
		udelay(50);			\
		__gpio_clear_pin(SPDA);		\
		__gpio_clear_pin(SPEN);		\
		udelay(50);			\
		value=((a<<8)|(b&0xFF));	\
		for(no=0;no<16;no++)		\
		{				\
			if((value&0x8000)==0x8000){	      \
				__gpio_set_pin(SPDA);}	      \
			else{				      \
				__gpio_clear_pin(SPDA); }     \
			udelay(50);			\
			__gpio_set_pin(SPCK);		\
			value=(value<<1);		\
			udelay(50);			\
			__gpio_clear_pin(SPCK);		\
		}					\
		__gpio_set_pin(SPEN);			\
		udelay(400);				\
	} while (0)
#define __spi_read_reg(reg,val)			\
	do{					\
		unsigned char no;		\
		unsigned short value;			\
		__gpio_as_output(SPEN); /* use SPDA */	\
		__gpio_as_output(SPCK); /* use SPCK */	\
		__gpio_as_output(SPDA); /* use SPDA */	\
		value = ((reg << 0) | (1 << 7));	\
		val = 0;				\
		__gpio_as_output(SPDA);			\
		__gpio_set_pin(SPEN);			\
		__gpio_clear_pin(SPCK);			\
		udelay(50);				\
		__gpio_clear_pin(SPDA);			\
		__gpio_clear_pin(SPEN);			\
		udelay(50);				\
		for (no = 0; no < 16; no++ ) {		\
			udelay(50);			\
			if(no < 8)			\
			{						\
				if (value & 0x80) /* send data */	\
					__gpio_set_pin(SPDA);		\
				else					\
					__gpio_clear_pin(SPDA);		\
				udelay(50);				\
				__gpio_set_pin(SPCK);			\
				value = (value << 1);			\
				udelay(50);				\
				__gpio_clear_pin(SPCK);			\
				if(no == 7)				\
					__gpio_as_input(SPDA);		\
			}						\
			else						\
			{						\
				udelay(100);				\
				__gpio_set_pin(SPCK);			\
				udelay(50);				\
				val = (val << 1);			\
				val |= __gpio_get_pin(SPDA);		\
				__gpio_clear_pin(SPCK);			\
			}						\
		}							\
		__gpio_as_output(SPDA);					\
		__gpio_set_pin(SPEN);					\
		udelay(400);						\
	} while(0)

#define __lcd_special_pin_init()		\
	do {						\
		__gpio_as_output0(LCD_RET);		\
		udelay(100);				\
		__gpio_as_output1(LCD_RET);		\
	} while (0)

#define __lcd_special_on()			     \
do {					     \
		udelay(50);			     \
		__gpio_as_output0(LCD_RET);	     \
		udelay(100);			     \
		__gpio_as_output1(LCD_RET);	     \
} while (0)

#define __lcd_special_off() \
do { \
	    __gpio_as_output0(LCD_RET);		\
} while (0)

#endif	/* CONFIG_JZLCD_AUO_A030FL01_V1 */

#ifndef __lcd_special_pin_init
#define __lcd_special_pin_init()
#endif
#ifndef __lcd_special_on
#define __lcd_special_on()
#endif
#ifndef __lcd_special_off
#define __lcd_special_off()
#endif


/*
 * Platform specific definition
 */
#if defined(CONFIG_JZ4810_VGA_DISPLAY)
#define __lcd_display_pin_init()
#define __lcd_display_on()
#define __lcd_display_off()
#elif defined(CONFIG_JZ4810_XXXX) 
#define __lcd_display_pin_init() \
do { \
	__gpio_as_output0(GPIO_LCD_VCC_EN_N);	\
	__lcd_special_pin_init();	   \
} while (0)

#define __lcd_display_on() \
do { \
	__gpio_as_output1(GPIO_LCD_VCC_EN_N);	 \
	__lcd_special_on();			\
} while (0)

#define __lcd_display_off() \
do { \
	__lcd_special_off(); \
	__gpio_as_output0(GPIO_LCD_VCC_EN_N);	 \
} while (0)

#else /* other boards */

#define __lcd_display_pin_init() \
do { \
	__lcd_special_pin_init();	   \
} while (0)

#define __lcd_display_on() \
do { \
	__lcd_special_on();			\
	__lcd_set_backlight_level(80);		\
} while (0)

#define __lcd_display_off() \
do { \
	__lcd_close_backlight();	   \
	__lcd_special_off();	 \
} while (0)

#endif /* LEPUS */


/*****************************************************************************
 * LCD display pin dummy macros
 *****************************************************************************/

#ifndef __lcd_display_pin_init
#define __lcd_display_pin_init()
#endif
#ifndef __lcd_slcd_special_on
#define __lcd_slcd_special_on()
#endif
#ifndef __lcd_display_on
#define __lcd_display_on()
#endif
#ifndef __lcd_display_off
#define __lcd_display_off()
#endif
#ifndef __lcd_set_backlight_level
#define __lcd_set_backlight_level(n)
#endif

#endif /* __JZ4810_LCD_H__ */
