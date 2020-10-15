#include <asm/jzsoc.h>

#if defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)/* board pavo */
	#include "../jz4760_lcd.h"
// #elif defined(CONFIG_JZ4770_F4770) || defined(CONFIG_JZ4770_PISCES)
	// #include "../jz4770_lcd.h"
#else
	#error "driver/video/jz4760_lcd, please define SPI pins on your board."
#endif

struct jz4760lcd_info jz4760_lcd_panel = {
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER |	// Underrun recover
			   LCD_CFG_NEWDES |							// 8words descriptor
			   LCD_CFG_MODE_GENERIC_TFT |				// General TFT panel
			   LCD_CFG_MODE_TFT_24BIT |					// output 18bpp
			   LCD_CFG_HSP |							// Hsync polarity: active low
			   LCD_CFG_VSP,								// Vsync polarity: leading edge is falling edge
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,		// 16words burst, enable out FIFO underrun irq
		640, 480, 54, 96, 2, 16, 48, 10, 33
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
		.fg0 = {32, 0, 0, 640, 480},		// bpp, x, y, w, h
		.fg1 = {32, 0, 0, 640, 480},		// bpp, x, y, w, h
	},
};
