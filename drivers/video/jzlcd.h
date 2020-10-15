/*
 * linux/drivers/video/jzlcd.h -- Ingenic On-Chip LCD frame buffer device
 *
 * Copyright (C) 2005-2007, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __JZLCD_H__
#define __JZLCD_H__

#include <asm/io.h>

#define NR_PALETTE	256

struct lcd_desc{
	unsigned int next_desc; /* LCDDAx */
	unsigned int databuf;   /* LCDSAx */
	unsigned int frame_id;  /* LCDFIDx */ 
	unsigned int cmd;       /* LCDCMDx */
};

#define MODE_MASK		0x0f
#define MODE_TFT_GEN		0x00
#define MODE_TFT_SHARP		0x01
#define MODE_TFT_CASIO		0x02
#define MODE_TFT_SAMSUNG	0x03
#define MODE_CCIR656_NONINT	0x04
#define MODE_CCIR656_INT	0x05
#define MODE_STN_COLOR_SINGLE	0x08
#define MODE_STN_MONO_SINGLE	0x09
#define MODE_STN_COLOR_DUAL	0x0a
#define MODE_STN_MONO_DUAL	0x0b
#define MODE_8BIT_SERIAL_TFT    0x0c

#define MODE_TFT_18BIT          (1<<7)

#define STN_DAT_PIN1	(0x00 << 4)
#define STN_DAT_PIN2	(0x01 << 4)
#define STN_DAT_PIN4	(0x02 << 4)
#define STN_DAT_PIN8	(0x03 << 4)
#define STN_DAT_PINMASK	STN_DAT_PIN8

#define STFT_PSHI	(1 << 15)
#define STFT_CLSHI	(1 << 14)
#define STFT_SPLHI	(1 << 13)
#define STFT_REVHI	(1 << 12)

#define SYNC_MASTER	(0 << 16)
#define SYNC_SLAVE	(1 << 16)

#define DE_P		(0 << 9)
#define DE_N		(1 << 9)

#define PCLK_P		(0 << 10)
#define PCLK_N		(1 << 10)

#define HSYNC_P		(0 << 11)
#define HSYNC_N		(1 << 11)

#define VSYNC_P		(0 << 8)
#define VSYNC_N		(1 << 8)

#define DATA_NORMAL	(0 << 17)
#define DATA_INVERSE	(1 << 17)


/* Jz LCDFB supported I/O controls. */
#define FBIOSETBACKLIGHT	0x4688
#define FBIODISPON		0x4689
#define FBIODISPOFF		0x468a
#define FBIORESET		0x468b
#define FBIOPRINT_REGS		0x468c
#define FBIOGETBUFADDRS		0x468d
#define FBIOROTATE		0x46a0 /* rotated fb */

struct jz_lcd_buffer_addrs_t {
	int fb_num;
	unsigned int lcd_desc_phys_addr;
	unsigned int fb_phys_addr[CONFIG_JZLCD_FRAMEBUFFER_MAX];
};


/*
 * LCD panel specific definition
 */
#if defined(CONFIG_JZLCD_TRULY_TFTG320240DTSW)

#if defined(CONFIG_JZ4730_PMP)
#define LCD_RESET_PIN	63
#endif

#define __lcd_special_on() \
do { \
	__gpio_set_pin(LCD_RESET_PIN); \
	__gpio_as_output(LCD_RESET_PIN); \
	__gpio_clear_pin(LCD_RESET_PIN); \
	udelay(100); \
	__gpio_set_pin(LCD_RESET_PIN); \
} while (0)

#endif /* CONFIG_JZLCD_TRULY_TFTG320240DTSW */

#if defined(CONFIG_JZLCD_SAMSUNG_LTV350QVF04)

#if defined(CONFIG_JZ4730_FPRINT)
#define PortSDI	60
#define PortSCL	61
#define PortCS	62
#define PortRST	63
#define PortSht 64
#endif

#if defined(CONFIG_JZ4730_GPS)
#define PortSDI	74
#define PortSCL	72
#define PortCS	73
#define PortRST	60
#define PortSht 59
#endif

#ifndef PortSDI
#define PortSDI	0
#endif
#ifndef PortSCL
#define PortSCL	0
#endif
#ifndef PortCS
#define PortCS	0
#endif
#ifndef PortRST
#define PortRST	0
#endif
#ifndef PortSht
#define PortSht	0
#endif

#define __lcd_special_pin_init() \
do { \
	__gpio_as_output(PortSDI); /* SDI */\
	__gpio_as_output(PortSCL); /* SCL */ \
	__gpio_as_output(PortCS); /* CS */ \
	__gpio_as_output(PortRST); /* Reset */ \
	__gpio_as_output(PortSht); /* Shut Down # */ \
	__gpio_set_pin(PortCS); \
	__gpio_set_pin(PortSCL); \
	__gpio_set_pin(PortSDI); \
} while (0)

#define __spi_out(val) \
do { \
	int __i__; \
	unsigned int _t_ = (val); \
	__gpio_clear_pin(PortCS); \
	udelay(25); \
	for (__i__ = 0; __i__ < 24; __i__++ ) { \
		__gpio_clear_pin(PortSCL); \
		if (_t_ & 0x800000) \
			__gpio_set_pin(PortSDI); \
		else \
			__gpio_clear_pin(PortSDI); \
		_t_ <<= 1; \
		udelay(25); \
		__gpio_set_pin(PortSCL); \
		udelay(25); \
	} \
	__gpio_set_pin(PortCS); \
	udelay(25); \
	__gpio_set_pin(PortSDI); \
	udelay(25); \
	__gpio_set_pin(PortSCL); \
} while (0)

#define __spi_id_op_data(rs, rw, val) \
	__spi_out((0x1d<<18)|((rs)<<17)|((rw)<<16)|(val))

#define __spi_write_reg(reg, val) \
do { \
	__spi_id_op_data(0, 0, (reg)); \
	__spi_id_op_data(1, 0, (val)); \
} while (0)

#define __lcd_special_on() \
do { \
	__gpio_set_pin(PortSht); \
	__gpio_clear_pin(PortRST); \
	mdelay(10); \
	__gpio_set_pin(PortRST); \
	mdelay(1); \
	__spi_write_reg(0x09, 0); \
	mdelay(10); \
	__spi_write_reg(0x09, 0x4000); \
	__spi_write_reg(0x0a, 0x2000); \
	mdelay(40); \
	__spi_write_reg(0x09, 0x4055); \
	mdelay(50); \
	__spi_write_reg(0x01, 0x409d); \
	__spi_write_reg(0x02, 0x0204); \
	__spi_write_reg(0x03, 0x0100); \
	__spi_write_reg(0x04, 0x3000); \
	__spi_write_reg(0x05, 0x4003); \
	__spi_write_reg(0x06, 0x000a); \
	__spi_write_reg(0x07, 0x0021); \
	__spi_write_reg(0x08, 0x0c00); \
	__spi_write_reg(0x10, 0x0103); \
	__spi_write_reg(0x11, 0x0301); \
	__spi_write_reg(0x12, 0x1f0f); \
	__spi_write_reg(0x13, 0x1f0f); \
	__spi_write_reg(0x14, 0x0707); \
	__spi_write_reg(0x15, 0x0307); \
	__spi_write_reg(0x16, 0x0707); \
	__spi_write_reg(0x17, 0x0000); \
	__spi_write_reg(0x18, 0x0004); \
	__spi_write_reg(0x19, 0x0000); \
	mdelay(60); \
	__spi_write_reg(0x09, 0x4a55); \
	__spi_write_reg(0x05, 0x5003); \
} while (0)

#define __lcd_special_off() \
do { \
	__spi_write_reg(0x09, 0x4055); \
	__spi_write_reg(0x05, 0x4003); \
	__spi_write_reg(0x0a, 0x0000); \
	mdelay(10); \
	__spi_write_reg(0x09, 0x4000); \
	__gpio_clear_pin(PortSht); \
} while (0)

#endif  /* CONFIG_JZLCD_SAMSUNG_LTV350QVF04 */

#if defined(CONFIG_JZLCD_AUO_A030FL01_V1)
#if defined(CONFIG_JZ4740_PAVO) /* board pavo */
	#define SPEN	(32*1+18)       /*LCD_CS*/
	#define SPCK	(32*1+17)       /*LCD_SCL*/
	#define SPDA	(32*2+12)       /*LCD_SDA*/
	#define LCD_RET (32*2+23)       /*use for lcd reset*/
#elif defined(CONFIG_JZ4740_LYRA) /* board lyra */
	#define SPEN	(32*3+19)       //LCD_CS
	#define SPCK	(32*3+18)       //LCD_SCL
	#define SPDA	(32*3+20)       //LCD_SDA
	#define LCD_RET (32*3+31)       //use for lcd reset
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
			udelay(400);			\
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
		__gpio_set_pin(SPCK);			\
		udelay(1);				\
		__gpio_clear_pin(SPDA);			\
		__gpio_clear_pin(SPEN);			\
		udelay(1);				\
		for (no = 0; no < 16; no++ ) {		\
			__gpio_clear_pin(SPCK);		\
			udelay(1);			\
			if(no < 8)			\
			{						\
				if (value & 0x80) /* send data */	\
					__gpio_set_pin(SPDA);		\
				else					\
					__gpio_clear_pin(SPDA);		\
				value = (value << 1);			\
				udelay(1);				\
				__gpio_set_pin(SPCK);			\
				udelay(1);				\
			}						\
			else						\
			{						\
				__gpio_as_input(SPDA);			\
				udelay(1);				\
				__gpio_set_pin(SPCK);			\
				udelay(1);				\
				val = (val << 1);			\
				val |= __gpio_get_pin(SPDA);		\
				udelay(1);				\
			}						\
			udelay(400);					\
		}							\
		__gpio_as_output(SPDA);					\
		__gpio_set_pin(SPEN);					\
		udelay(400);						\
	} while(0)
	
#define __lcd_special_pin_init()		\
	do {						\
		__gpio_as_output(SPEN); /* use SPDA */	\
		__gpio_as_output(SPCK); /* use SPCK */	\
		__gpio_as_output(SPDA); /* use SPDA */	\
		__gpio_as_output(LCD_RET);		\
		udelay(50);				\
		__gpio_clear_pin(LCD_RET);		\
		udelay(100);				\
		__gpio_set_pin(LCD_RET);		\
	} while (0)
#define __lcd_special_on()			     \
	do {					     \
		udelay(50);			     \
		__gpio_clear_pin(LCD_RET);	     \
		udelay(100);			     \
		__gpio_set_pin(LCD_RET);	     \
		__spi_write_reg(0x0D, 0x44);	     \
		__spi_write_reg(0x0D, 0x4D);	     \
		__spi_write_reg(0x0B, 0x06);	     \
		__spi_write_reg(0x40, 0xC0);	     \
		__spi_write_reg(0x42, 0x43);	     \
		__spi_write_reg(0x44, 0x28);	     \
		__spi_write_reg(0x0D, 0x4F);	     \
} while (0)

	#define __lcd_special_off() \
	do { \
		__spi_write_reg(0x04, 0x4C); \
	} while (0)

#endif	/* CONFIG_JZLCD_AUO_A030FL01_V1 */

//#if defined(CONFIG_JZLCD_FOXCONN_PT035TN01)
#if defined(CONFIG_JZLCD_FOXCONN_PT035TN01) || defined(CONFIG_JZLCD_INNOLUX_PT035TN01_SERIAL)

#if defined(CONFIG_JZLCD_FOXCONN_PT035TN01) /* board pmp */
#define MODE 0xcd 		/* 24bit parellel RGB */
#endif
#if defined(CONFIG_JZLCD_INNOLUX_PT035TN01_SERIAL)
#define MODE 0xc9		/* 8bit serial RGB */
#endif

#if defined(CONFIG_JZ4730_PMP)
	#define SPEN	60       //LCD_SPL
	#define SPCK	61       //LCD_CLS
	#define SPDA	62       //LCD_PS
	#define LCD_RET 63       //LCD_REV  //use for lcd reset
#elif defined(CONFIG_JZ4740_LEO) /* board leo */
	#define SPEN	(32*1+18)       //LCD_SPL
	#define SPCK	(32*1+17)       //LCD_CLS
	#define SPDA	(32*2+22)       //LCD_PS
	#define LCD_RET (32*2+23)       //LCD_REV  //use for lcd reset
#elif defined(CONFIG_JZ4740_PAVO) /* board pavo */
	#define SPEN	(32*1+18)       //LCD_SPL
	#define SPCK	(32*1+17)       //LCD_CLS
	#define SPDA	(32*2+12)       //LCD_D12
	#define LCD_RET (32*2+23)       //LCD_REV, GPC23
#if 0  /*old driver*/
	#define SPEN	(32*1+18)       //LCD_SPL
	#define SPCK	(32*1+17)       //LCD_CLS
	#define SPDA	(32*2+12)       //LCD_D12
	#define LCD_RET (32*3+27)       //PWM4  //use for lcd reset
#endif
#else
#error "driver/video/Jzlcd.h, please define SPI pins on your board."
#endif

	#define __spi_write_reg1(reg, val) \
	do { \
		unsigned char no;\
		unsigned short value;\
		unsigned char a=0;\
		unsigned char b=0;\
		a=reg;\
		b=val;\
		__gpio_set_pin(SPEN);\
		__gpio_set_pin(SPCK);\
		__gpio_clear_pin(SPDA);\
		__gpio_clear_pin(SPEN);\
		udelay(25);\
		value=((a<<8)|(b&0xFF));\
		for(no=0;no<16;no++)\
		{\
			__gpio_clear_pin(SPCK);\
			if((value&0x8000)==0x8000)\
			__gpio_set_pin(SPDA);\
			else\
			__gpio_clear_pin(SPDA);\
			udelay(25);\
			__gpio_set_pin(SPCK);\
			value=(value<<1); \
			udelay(25);\
		 }\
		__gpio_set_pin(SPEN);\
		udelay(100);\
	} while (0)

	#define __spi_write_reg(reg, val) \
	do {\
		__spi_write_reg1((reg<<2|2), val); \
		udelay(100); \
	}while(0)
	
	#define __lcd_special_pin_init() \
	do { \
		__gpio_as_output(SPEN); /* use SPDA */\
		__gpio_as_output(SPCK); /* use SPCK */\
		__gpio_as_output(SPDA); /* use SPDA */\
		__gpio_as_output(LCD_RET);\
		udelay(50);\
		__gpio_clear_pin(LCD_RET);\
		mdelay(150);\
		__gpio_set_pin(LCD_RET);\
	} while (0)

	#define __lcd_special_on() \
	do { \
		udelay(50);\
		__gpio_clear_pin(LCD_RET);\
		mdelay(150);\
		__gpio_set_pin(LCD_RET);\
		mdelay(10);\
		__spi_write_reg(0x00, 0x03); \
		__spi_write_reg(0x01, 0x40); \
		__spi_write_reg(0x02, 0x11); \
		__spi_write_reg(0x03, MODE); /* mode */ \
		__spi_write_reg(0x04, 0x32); \
		__spi_write_reg(0x05, 0x0e); \
		__spi_write_reg(0x07, 0x03); \
		__spi_write_reg(0x08, 0x08); \
		__spi_write_reg(0x09, 0x32); \
		__spi_write_reg(0x0A, 0x88); \
		__spi_write_reg(0x0B, 0xc6); \
		__spi_write_reg(0x0C, 0x20); \
		__spi_write_reg(0x0D, 0x20); \
	} while (0)	//reg 0x0a is control the display direction:DB0->horizontal level DB1->vertical level

/*		__spi_write_reg(0x02, 0x03); \
		__spi_write_reg(0x06, 0x40); \
		__spi_write_reg(0x0a, 0x11); \
		__spi_write_reg(0x0e, 0xcd); \
		__spi_write_reg(0x12, 0x32); \
		__spi_write_reg(0x16, 0x0e); \
		__spi_write_reg(0x1e, 0x03); \
		__spi_write_reg(0x22, 0x08); \
		__spi_write_reg(0x26, 0x40); \
		__spi_write_reg(0x2a, 0x88); \
		__spi_write_reg(0x2e, 0x88); \
		__spi_write_reg(0x32, 0x20); \
		__spi_write_reg(0x36, 0x20); \
*/
//	} while (0)	//reg 0x0a is control the display direction:DB0->horizontal level DB1->vertical level
	
	#define __lcd_special_off() \
	do { \
		__spi_write_reg(0x00, 0x03); \
	} while (0)

#endif	/* CONFIG_JZLCD_FOXCONN_PT035TN01 or CONFIG_JZLCD_INNOLUX_PT035TN01_SERIAL */


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

#if defined(CONFIG_JZ4730_GPS)

#define __lcd_set_backlight_level(n) \
do { \
	; \
} while (0)

#define __lcd_display_pin_init() \
do { \
	__lcd_special_pin_init(); \
	__gpio_as_output(94); /* PWM0 pin */ \
	__gpio_as_output(95); /* PWM1 pin */ \
} while (0)

#define __lcd_display_on() \
do { \
	__lcd_special_on(); \
	__gpio_set_pin(94); /* PWM0 pin */ \
	__gpio_set_pin(95); /* PWM1 pin */ \
	__lcd_set_backlight_level(8); \
} while (0)

#define __lcd_display_off() \
do { \
	__lcd_special_off(); \
} while (0)

#endif /* CONFIG_JZ4730_GPS */

#if defined(CONFIG_JZ4730_FPRINT)

#define __lcd_set_backlight_level(n) \
do { \
	REG_PWM_DUT(0) = n; \
	REG_PWM_PER(0) = 7; \
	REG_PWM_CTR(0) = 0x81; \
} while (0)

#if defined(CONFIG_JZLCD_FOXCONN_PT035TN01)
		
#define __lcd_display_pin_init() \
do { \
	__lcd_special_pin_init();\
	__gpio_as_pwm();\
	__lcd_set_backlight_level(8);\
} while (0)

#define __lcd_display_on() \
do { \
	__lcd_set_backlight_level(8); \
	__lcd_special_on();\
} while (0)

#define __lcd_display_off() \
do { \
	__lcd_set_backlight_level(0); \
	__lcd_special_off();\
} while (0)

#else
	
#define __lcd_display_pin_init() \
do { \
	__gpio_as_output(GPIO_DISP_OFF_N); \
	__gpio_as_pwm(); \
	__lcd_set_backlight_level(8); \
} while (0)

#define __lcd_display_on() \
do { \
	__lcd_set_backlight_level(8); \
	__gpio_set_pin(GPIO_DISP_OFF_N); \
} while (0)
	
#define __lcd_display_off() \
do { \
	__lcd_set_backlight_level(0); \
	__gpio_clear_pin(GPIO_DISP_OFF_N); \
} while (0)
#endif

#endif /* CONFIG_JZ4730_FPRINT */

#if defined(CONFIG_JZ4730_LIBRA)

#define __lcd_set_backlight_level(n) \
do { \
} while (0)

#define __lcd_display_pin_init() \
do { \
	__lcd_special_pin_init(); \
	__gpio_clear_pin(100); \
	__gpio_as_output(100); \
	__gpio_as_output(94); \
	__gpio_as_output(95); \
	__lcd_set_backlight_level(8); \
} while (0)

#define __lcd_display_on() \
do { \
	__lcd_special_on(); \
	__gpio_set_pin(100); \
	__gpio_set_pin(94); \
	__gpio_set_pin(95); \
} while (0)

#define __lcd_display_off() \
do { \
	__lcd_special_off(); \
	__gpio_clear_pin(100); \
	__gpio_clear_pin(94); \
	__gpio_clear_pin(95); \
} while (0)

#endif /* CONFIG_JZ4730_LIBRA */

#if defined(CONFIG_JZ4730_PMP)

#define __lcd_set_backlight_level(n) \
do { \
	REG_PWM_DUT(0) = n; \
	REG_PWM_PER(0) = 7; \
	REG_PWM_CTR(0) = 0x81; \
} while (0)

#define __lcd_display_pin_init() \
do { \
	__gpio_as_output(GPIO_DISP_OFF_N); \
	__gpio_as_pwm(); \
	__lcd_set_backlight_level(10); \
	__lcd_special_pin_init(); \
} while (0)

#define __lcd_display_on() \
do { \
	__lcd_special_on(); \
	__lcd_set_backlight_level(8); \
	__gpio_set_pin(GPIO_DISP_OFF_N); \
} while (0)

#define __lcd_display_off() \
do { \
	__lcd_special_off(); \
	__lcd_set_backlight_level(0); \
	__gpio_clear_pin(GPIO_DISP_OFF_N); \
} while (0)

#endif /* CONFIG_JZ4730_PMP */

/*#if defined(CONFIG_JZ4740_LEO) || defined(CONFIG_JZ4740_PAVO)*/
#if defined(CONFIG_SOC_JZ4740)
#if defined(CONFIG_JZ4740_PAVO) || defined(CONFIG_JZ4740_LYRA)
#define GPIO_PWM    123		/* GP_D27 */
#define PWM_CHN 4    /* pwm channel */
#define PWM_FULL 101
/* 100 level: 0,1,...,100 */
#define __lcd_set_backlight_level(n)\
do { \
__gpio_as_output(32*3+27); \
__gpio_set_pin(32*3+27); \
} while (0)

#define __lcd_close_backlight() \
do { \
__gpio_as_output(GPIO_PWM); \
__gpio_clear_pin(GPIO_PWM); \
} while (0)

#elif defined(CONFIG_JZ4720_VIRGO)
#define GPIO_PWM    119		/* GP_D23 */
#define PWM_CHN 0    /* pwm channel */
#define PWM_FULL 101
/* 100 level: 0,1,...,100 */
/*#define __lcd_set_backlight_level(n)                     \
do {                                                     \
	__gpio_as_pwm(0); \
        __tcu_disable_pwm_output(PWM_CHN);               \
        __tcu_stop_counter(PWM_CHN);                     \
        __tcu_init_pwm_output_high(PWM_CHN);             \
        __tcu_set_pwm_output_shutdown_abrupt(PWM_CHN);   \
        __tcu_select_clk_div1(PWM_CHN);                  \
        __tcu_mask_full_match_irq(PWM_CHN);              \
        __tcu_mask_half_match_irq(PWM_CHN);              \
        __tcu_set_count(PWM_CHN,0);                      \
        __tcu_set_full_data(PWM_CHN,__cpm_get_extalclk()/1000);           \
        __tcu_set_half_data(PWM_CHN,__cpm_get_extalclk()/1000*n/100);     \
        __tcu_enable_pwm_output(PWM_CHN);                \
        __tcu_select_extalclk(PWM_CHN);                  \
        __tcu_start_counter(PWM_CHN);                    \
} while (0)
*/

#define __lcd_set_backlight_level(n)			\
do {							\
       __gpio_as_output(GPIO_PWM);			\
       __gpio_set_pin(GPIO_PWM);			\
} while (0)

#define __lcd_close_backlight() \
do { \
__gpio_as_output(GPIO_PWM); \
__gpio_clear_pin(GPIO_PWM); \
} while (0)

#else
#define __lcd_set_backlight_level(n)
#define __lcd_close_backlight()

#endif /* #if defined(CONFIG_MIPS_JZ4740_PAVO) */

#define __lcd_display_pin_init() \
do { \
	__gpio_as_output(GPIO_DISP_OFF_N); \
	__cpm_start_tcu(); \
	__lcd_special_pin_init(); \
} while (0)
/*	__lcd_set_backlight_level(100); \*/
#define __lcd_display_on() \
do { \
	__gpio_set_pin(GPIO_DISP_OFF_N); \
	__lcd_special_on(); \
	__lcd_set_backlight_level(80); \
} while (0)

#define __lcd_display_off() \
do { \
	__lcd_special_off(); \
	__lcd_close_backlight(); \
	__gpio_clear_pin(GPIO_DISP_OFF_N); \
} while (0)

#endif /* CONFIG_MIPS_JZ4740_LEO */

#if defined(CONFIG_JZLCD_MSTN_240x128)

#if 0	/* The final version does not use software emulation of VCOM. */

#define GPIO_VSYNC	59
#define GPIO_VCOM	90

#define REG_VCOM	REG_GPIO_GPDR((GPIO_VCOM>>5))
#define VCOM_BIT	(1 << (GPIO_VCOM & 0x1f))
static unsigned int vcom_static;
static void vsync_irq(int irq, void *dev_id, struct pt_regs *reg)
{
	vcom_static = REG_VCOM;
	vcom_static ^= VCOM_BIT;
	REG_VCOM = vcom_static;
}

#define __lcd_display_pin_init()					  \
	__gpio_as_irq_rise_edge(GPIO_VSYNC);				  \
	__gpio_as_output(GPIO_VCOM);					  \
	{								  \
	static int inited = 0;						  \
	if (!inited) {							  \
	inited = 1;							  \
	if (request_irq(IRQ_GPIO_0 + GPIO_VSYNC, vsync_irq, SA_INTERRUPT, \
			"vsync", 0)) {					  \
		err = -EBUSY;						  \
		goto failed;						  \
	}}}

#endif

/* We uses AC BIAs pin to generate VCOM signal, so above code should be removed.
 */
#endif
/*****************************************************************************
 * LCD display pin dummy macros
 *****************************************************************************/
#ifndef __lcd_display_pin_init
#define __lcd_display_pin_init()
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

#endif /* __JZLCD_H__ */
