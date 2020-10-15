#include <asm/jzsoc.h>

#if defined(CONFIG_JZ4760_LCD_FOXCONN_PT035TN01) /* board FUWA */
	#define MODE 0xcd 		/* 24bit parellel RGB */
#elif defined(CONFIG_JZ4760_LCD_INNOLUX_PT035TN01_SERIAL)
	#define MODE 0xc9		/* 8bit serial RGB */
#endif

#if defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)/* board FuWa */
	#include "../jz4760_lcd.h"
	#define SPEN		(32*2+29)       /*LCD_CS*/
	#define SPCK		(32*2+28)       /*LCD_SCL*/
	#define SPDA		(32*2+21)       /*LCD_SDA*/
	#define LCD_RET 	(32*5+6)       /*LCD_DISP_N use for lcd reset*/
#elif CONFIG_JZ4760_CYGNUS /* board cygnus */
	#define SPEN            (32*3+13)       /*LCD_CS GPD13*/
	#define SPCK            (32*4+13)       /*LCD_SCL GPE13*/
	#define SPDA            (32*1+29)       /*LCD_SDA GPB29*/
	#define LCD_DISP_N      (32*4+11)       /*LCD_DISP_N use for lcd reset*/
	#define LCD_RET         (32*4+12)       /*LCD_RESET use for lcd reset*/
#else
	#error "driver/video/jz4760_lcd, please define SPI pins on your board."
#endif

#define __spi_write_reg1(reg, val) \
	do { \
		unsigned char no; \
		unsigned short value; \
		unsigned char a=0; \
		unsigned char b=0; \
		a=reg; \
		b=val; \
		__gpio_set_pin(SPEN); \
		__gpio_set_pin(SPCK); \
		__gpio_clear_pin(SPDA); \
		__gpio_clear_pin(SPEN); \
		udelay(25); \
		value=((a<<8)|(b&0xFF)); \
		for(no=0;no<16;no++) \
		{\
			__gpio_clear_pin(SPCK); \
			if((value&0x8000)==0x8000) \
			__gpio_set_pin(SPDA); \
			else\
			__gpio_clear_pin(SPDA); \
			udelay(25); \
			__gpio_set_pin(SPCK); \
			value=(value<<1); \
			udelay(25); \
		 }\
		__gpio_set_pin(SPEN); \
		udelay(100); \
	} while (0)

#define __spi_write_reg(reg, val) \
	do {\
		__spi_write_reg1((reg<<2|2), val); \
		udelay(100); \
	}while(0)

#define __lcd_special_pin_init() \
	do { \
		__gpio_as_output(SPEN); /* use SPDA */ \
		__gpio_as_output(SPCK); /* use SPCK */ \
		__gpio_as_output(SPDA); /* use SPDA */ \
		__gpio_as_output(LCD_RET); \
		udelay(50); \
		__gpio_clear_pin(LCD_RET); \
		mdelay(150); \
		__gpio_set_pin(LCD_RET); \
	} while (0)

#define __lcd_special_on() \
	do { \
		udelay(50); \
		__gpio_clear_pin(LCD_RET); \
		mdelay(150); \
		__gpio_set_pin(LCD_RET); \
		mdelay(10); \
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

struct jz4760lcd_info jz4760_lcd_panel = {
#if defined(CONFIG_JZ4760_LCD_FOXCONN_PT035TN01) /* board FUWA */
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER |	// Underrun recover
			   LCD_CFG_NEWDES |							// 8words descriptor
			   LCD_CFG_MODE_GENERIC_TFT |				// General TFT panel
			   // LCD_CFG_MODE_TFT_18BIT | 				// output 18bpp
			   LCD_CFG_MODE_TFT_24BIT |					// output 24bpp
			   LCD_CFG_HSP |							// Hsync polarity: active low
			   LCD_CFG_VSP |							// Vsync polarity: leading edge is falling edge
			   LCD_CFG_PCP,								// Pix-CLK polarity: data translations at falling/edge
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,	// 16words burst, enable out FIFO underrun irq
		320, 240, 80, 1, 1, 10, 50, 10, 13
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
		.ipu_restart = 0x80001000,			// ipu restart
		.fg_change = FG_CHANGE_ALL,			// change all initially
		.fg0 = {32, 0, 0, 320, 240},		// bpp, x, y, w, h
		.fg1 = {32, 0, 0, 320, 240},		// bpp, x, y, w, h
#elif defined(CONFIG_JZ4760_LCD_INNOLUX_PT035TN01_SERIAL)
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER |	// Underrun recover
			   LCD_CFG_NEWDES |							// 8words descriptor
			   LCD_CFG_MODE_SERIAL_TFT |				// Serial TFT panel
			   LCD_CFG_MODE_TFT_18BIT |					// output 18bpp
			   LCD_CFG_HSP |							// Hsync polarity: active low
			   LCD_CFG_VSP |							// Vsync polarity: leading edge is falling edge
			   LCD_CFG_PCP,								// Pix-CLK polarity: data translations at falling/edge
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,		// 16words burst, enable out FIFO underrun irq
		320, 240, 60, 1, 1, 10, 50, 10, 13
	},
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN |			// Use OSD mode
					// LCD_OSDC_ALPHAEN |	// enable alpha
				   LCD_OSDC_F0EN,			// enable Foreground0
		.osd_ctrl = 0,						// disable ipu
		.rgb_ctrl = 0,
		.bgcolor = 0x000000,				// set background color Black
		.colorkey0 = 0,						// disable colorkey
		.colorkey1 = 0,						// disable colorkey
		.alpha = 0xA0,						// alpha value
		.ipu_restart = 0x80001000,			// ipu restart
		.fg_change = FG_CHANGE_ALL,			// change all initially
		.fg0 = {32, 0, 0, 320, 240},		// bpp, x, y, w, h
		.fg1 = {32, 0, 0, 320, 240},		// bpp, x, y, w, h
	},
#endif
};
