#include <asm/jzsoc.h>

#if defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)/* board pavo */
	#include "../jz4760_lcd.h"
// #elif defined(CONFIG_JZ4770_F4770) || defined(CONFIG_JZ4770_PISCES)
	// #include "../jz4770_lcd.h"
#else
	#error "Define special lcd pins for your platform."
#endif

struct jz4760lcd_info jz4760_lcd_panel = {
 .panel = {
	 // .cfg = LCD_CFG_LCDPIN_SLCD | LCD_CFG_RECOVER | // Underrun recover
	 .cfg = LCD_CFG_LCDPIN_SLCD |  // Underrun recover
			 // LCD_CFG_DITHER |   // dither
			LCD_CFG_NEWDES |     // 8words descriptor
			LCD_CFG_MODE_SLCD,   // TFT Smart LCD panel
	 .slcd_cfg = SLCD_CFG_DWIDTH_18BIT | SLCD_CFG_CWIDTH_18BIT | SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW | SLCD_CFG_CLK_ACTIVE_FALLING | SLCD_CFG_TYPE_PARALLEL,
	 .ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,  // 16words burst, enable out FIFO underrun irq
	 400, 240, 60, 0, 0, 0, 0, 0, 0,
 },
 .osd = {
	 .osd_cfg = LCD_OSDC_OSDEN |     // Use OSD mode
			// LCD_OSDC_ALPHAEN |  // enable alpha
			// LCD_OSDC_ALPHAMD |  // alpha blending mode
			LCD_OSDC_F0EN,     // enable Foreground0
			// LCD_OSDC_F1EN,    // enable Foreground1
		.osd_ctrl = 0,            // disable ipu
		.rgb_ctrl = 0,
		.bgcolor = 0x000000,        // set background color Black
		.colorkey0 = 0,           // disable colorkey
		.colorkey1 = 0,           // disable colorkey
		.alpha = 0xA0,            // alpha value
		.ipu_restart = 0x80001000,      // ipu restart
		.fg_change = FG_CHANGE_ALL,     // change all initially
		// .fg0 = {32, 0, 0, 400, 240},   // bpp, x, y, w, h
		.fg0 = {32, 0, 0, 320, 240},    // bpp, x, y, w, h
		.fg1 = {32, 0, 0, 400, 240},    // bpp, x, y, w, h
	},
};
