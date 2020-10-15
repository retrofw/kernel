#include <asm/jzsoc.h>

#if defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)/* board pavo */
	#include "../jz4760_lcd.h"
	#define SPEN		(32*1+29)       /*LCD_CS*/
	#define SPCK		(32*1+28)       /*LCD_SCL*/
	#define SPDA		(32*1+21)       /*LCD_SDA*/
	#define LCD_RET 	(32*5+6)       /*LCD_DISP_N use for lcd reset*/
#elif defined(CONFIG_JZ4770_F4770) || defined(CONFIG_JZ4770_PISCES)
	#include "../jz4770_lcd.h"
	#define SPEN		(32*4+16)       /*LCD_CS*/
	#define SPCK		(32*4+15)       /*LCD_SCL*/
	#define SPDA		(32*4+17)       /*LCD_SDA*/
	#define LCD_RET 	(32*4+2)       /*LCD_DISP_N use for lcd reset*/
#else
#error "driver/video/jz4760_lcd.h: define SPI pins on your board"
#endif

#define __spi_write_reg(reg, val) \
	do { \
		unsigned char no; \
		unsigned short value; \
		unsigned char a=0; \
		unsigned char b=0; \
		__gpio_as_output(SPEN); /* use SPDA */ \
		__gpio_as_output(SPCK); /* use SPCK */ \
		__gpio_as_output(SPDA); /* use SPDA */ \
		a=reg; \
		b=val; \
		__gpio_set_pin(SPEN); \
		__gpio_clear_pin(SPCK); \
		udelay(50); \
		__gpio_clear_pin(SPDA); \
		__gpio_clear_pin(SPEN); \
		udelay(50); \
		value=((a<<8)|(b&0xFF)); \
		for(no=0;no<16;no++) \
		{ \
			if((value&0x8000)==0x8000){ \
				__gpio_set_pin(SPDA);} \
			else{ \
				__gpio_clear_pin(SPDA); } \
			udelay(50); \
			__gpio_set_pin(SPCK); \
			value=(value<<1); \
			udelay(50); \
			__gpio_clear_pin(SPCK); \
		} \
		__gpio_set_pin(SPEN); \
		udelay(400); \
	} while (0)
#define __spi_read_reg(reg,val) \
	do{ \
		unsigned char no; \
		unsigned short value; \
		__gpio_as_output(SPEN); /* use SPDA */ \
		__gpio_as_output(SPCK); /* use SPCK */ \
		__gpio_as_output(SPDA); /* use SPDA */ \
		value = ((reg << 0) | (1 << 7)); \
		val = 0; \
		__gpio_as_output(SPDA); \
		__gpio_set_pin(SPEN); \
		__gpio_clear_pin(SPCK); \
		udelay(50); \
		__gpio_clear_pin(SPDA); \
		__gpio_clear_pin(SPEN); \
		udelay(50); \
		for (no = 0; no < 16; no++ ) { \
			udelay(50); \
			if(no < 8) \
			{ \
				if (value & 0x80) /* send data */ \
					__gpio_set_pin(SPDA); \
				else \
					__gpio_clear_pin(SPDA); \
				udelay(50); \
				__gpio_set_pin(SPCK); \
				value = (value << 1); \
				udelay(50); \
				__gpio_clear_pin(SPCK); \
				if(no == 7) \
					__gpio_as_input(SPDA); \
			} \
			else \
			{ \
				udelay(100); \
				__gpio_set_pin(SPCK); \
				udelay(50); \
				val = (val << 1); \
				val |= __gpio_get_pin(SPDA); \
				__gpio_clear_pin(SPCK); \
			} \
		} \
		__gpio_as_output(SPDA); \
		__gpio_set_pin(SPEN); \
		udelay(400); \
	} while(0)

#define __lcd_special_pin_init() \
	do { \
		__gpio_as_output(SPEN); /* use SPDA */ \
		__gpio_as_output(SPCK); /* use SPCK */ \
		__gpio_as_output(SPDA); /* use SPDA */ \
		__gpio_as_output(LCD_RET); \
		udelay(50); \
		__gpio_clear_pin(LCD_RET); \
		udelay(100); \
		__gpio_set_pin(LCD_RET); \
	} while (0)
#define __lcd_special_on() \
	do { \
		udelay(50); \
		__gpio_clear_pin(LCD_RET); \
		udelay(100); \
		__gpio_set_pin(LCD_RET); \
} while (0)

#define __lcd_special_off() \
	do { \
		__gpio_clear_pin(LCD_RET); \
	} while (0)


struct jz4760lcd_info jz4760_lcd_panel = {
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER |	// Underrun recover
			   LCD_CFG_NEWDES |							// 8words descriptor
			   LCD_CFG_MODE_GENERIC_TFT |				// General TFT panel
			   LCD_CFG_MODE_TFT_24BIT |					// output 18bpp
			   LCD_CFG_HSP |							// Hsync polarity: active low
			   LCD_CFG_VSP,								// Vsync polarity: leading edge is falling edge
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	// 16words burst, enable out FIFO underrun irq
		480, 272, 60, 41, 10, 8, 4, 4, 2,
	},
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN |			// Use OSD mode
				   // LCD_OSDC_ALPHAEN |	// enable alpha
				   LCD_OSDC_F0EN,			// enable Foreground0
				   // LCD_OSDC_F1EN,		// enable Foreground1
		.osd_ctrl = 0,						// disable ipu
		.rgb_ctrl = 0,
		.bgcolor = 0x000000,				// set background color Black
		.colorkey0 = 0,						// disable colorkey
		.colorkey1 = 0,						// disable colorkey
		.alpha = 0xA0,						// alpha value
		.ipu_restart = 0x80001000,  		// ipu restart
		.fg_change = FG_CHANGE_ALL, 		// change all initially
		.fg0 = {16, 0, 0, 480, 272},		// bpp, x, y, w, h
		.fg1 = {16, 0, 0, 720, 573},		// bpp, x, y, w, h
	},
};
