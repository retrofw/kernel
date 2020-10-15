#include <asm/jzsoc.h>

#if defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)/* board pavo */
	#include "../jz4760_lcd.h"
	#define SPEN		(32*1+29)       /*LCD_CS*/
	#define SPCK		(32*1+28)       /*LCD_SCL*/
	#define SPDA		(32*1+21)       /*LCD_SDA*/
	#define LCD_RET 	(32*5+6)       /*LCD_DISP_N use for lcd reset*/
// #elif defined(CONFIG_JZ4770_F4770) || defined(CONFIG_JZ4770_PISCES)
	// #include "../jz4770_lcd.h"
#else
	#error "driver/video/jz4760_lcd, please define SPI pins on your board."
#endif

#if 0
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
		udelay(500); \
		__gpio_clear_pin(SPDA); \
		__gpio_clear_pin(SPEN); \
		udelay(500); \
		value=((a<<10)|(b&0xFF)); \
		for(no=0;no<16;no++) \
		{ \
			if((value&0x8000)==0x8000){ \
				__gpio_set_pin(SPDA);} \
			else{ \
				__gpio_clear_pin(SPDA); } \
			udelay(500); \
			__gpio_set_pin(SPCK); \
			value=(value<<1); \
			udelay(500); \
			__gpio_clear_pin(SPCK); \
		} \
		__gpio_set_pin(SPEN); \
		udelay(4000); \
	} while (0)
#define __spi_read_reg(reg,val) \
	do{ \
		unsigned char no; \
		unsigned short value; \
		__gpio_as_output(SPEN); /* use SPDA */ \
		__gpio_as_output(SPCK); /* use SPCK */ \
		__gpio_as_output(SPDA); /* use SPDA */ \
		value = ((reg << 2) | (1 << 1)); \
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

#endif

#define __lcd_special_pin_init() \
	do { \
		__gpio_as_output(SPEN); /* use SPDA */ \
		__gpio_as_output(SPCK); /* use SPCK */ \
		__gpio_as_output(SPDA); /* use SPDA */ \
		__gpio_set_pin(SPEN); \
		__gpio_as_output(LCD_RET); \
		udelay(500); \
		__gpio_clear_pin(LCD_RET); \
		udelay(1000); \
		__gpio_set_pin(LCD_RET); \
		udelay(1000); \
	} while (0)

#if 0
#define __lcd_special_on() \
	do { \
		udelay(1000); \
		__spi_write_reg(0x02, 0x07); \
		__spi_write_reg(0x03, 0x5F); \
		__spi_write_reg(0x04, 0x17); \
		__spi_write_reg(0x05, 0x20); \
		__spi_write_reg(0x06, 0x08); \
		__spi_write_reg(0x07, 0x20); \
		__spi_write_reg(0x08, 0x20); \
		__spi_write_reg(0x09, 0x20); \
		__spi_write_reg(0x0A, 0x20); \
		__spi_write_reg(0x0B, 0x20); \
		__spi_write_reg(0x0C, 0x20); \
		__spi_write_reg(0x0D, 0x22); \
		__spi_write_reg(0x0E, 0x2F); \
		__spi_write_reg(0x0F, 0x2f); \
		__spi_write_reg(0x10, 0x2F); \
		__spi_write_reg(0x11, 0x15); \
		__spi_write_reg(0x12, 0xaa); \
		__spi_write_reg(0x13, 0xFF); \
		__spi_write_reg(0x14, 0x86); \
		__spi_write_reg(0x15, 0x8e); \
		__spi_write_reg(0x16, 0xd6); \
		__spi_write_reg(0x17, 0xfe); \
		__spi_write_reg(0x18, 0x28); \
		__spi_write_reg(0x19, 0x52); \
		__spi_write_reg(0x1A, 0x7c); \
		__spi_write_reg(0x1B, 0xe9); \
		__spi_write_reg(0x1C, 0x42); \
		__spi_write_reg(0x1D, 0x88); \
		__spi_write_reg(0x1E, 0xb8); \
		__spi_write_reg(0x1F, 0xff); \
		__spi_write_reg(0x20, 0xf0); \
		__spi_write_reg(0x21, 0xf0); \
		__spi_write_reg(0x22, 0x08); \
	} while (0)
#endif

#define __lcd_special_off() \
	do { \
		__gpio_clear_pin(LCD_RET); \
		__gpio_as_input(LCD_RET); \
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
		800, 480, 60, 1, 1, 40, 215, 10, 34,
	},
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN |			// Use OSD mode
				   // LCD_OSDC_ALPHAEN |	// enable alpha
				   LCD_OSDC_F0EN,			// enable Foreground0
				   // LCD_OSDC_F1EN,		// enable Foreground1
		.osd_ctrl = 0,						// disable ipu
		.rgb_ctrl = 0,
		.bgcolor = 0xff,					// set background color Black
		.colorkey0 = 0,						// disable colorkey
		.colorkey1 = 0,						// disable colorkey
		.alpha = 0xA0,						// alpha value
		.ipu_restart = 0x80001000,			// ipu restart
		.fg_change = FG_CHANGE_ALL,			// change all initially
		.fg0 = {32, 0, 0, 800, 480},		// bpp, x, y, w, h
		.fg1 = {32, 0, 0, 800, 480},		// bpp, x, y, w, h
	},
};
