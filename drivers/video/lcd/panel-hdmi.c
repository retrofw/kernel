#include <asm/jzsoc.h>

#if defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)/* board pavo */
	#include "../jz4760_lcd.h"
// #elif defined(CONFIG_JZ4770_F4770) || defined(CONFIG_JZ4770_PISCES)
	// #include "../jz4770_lcd.h"
#else
	#error "driver/video/jz4760_lcd, please define SPI pins on your board."
#endif

#define AIC_FR_TFTH_BIT 16
#define AIC_FR_RFTH_BIT 24

#define PANEL_MODE_HDMI_480P 3
#define PANEL_MODE_HDMI_576P 4
#define PANEL_MODE_HDMI_720P50 5
#define PANEL_MODE_HDMI_720P60 6

struct jz4760lcd_info jz4760_info_hdmi_480p = {
	.panel = {
		.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
			   LCD_CFG_NEWDES | LCD_CFG_RECOVER |
			   LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_BST_32,
		/* width,height,freq,hsync,vsync,elw,blw,efw,bfw */
		640, 480, 60, 96, 2, 48, 16, 33, 10,	//HDMI-480P
	},
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN |	// Use OSD mode
				   LCD_OSDC_ALPHAEN |	// enable alpha
				   LCD_OSDC_F0EN,		// enable Foreground0
				   // LCD_OSDC_F1EN,	// enable Foreground1
		.osd_ctrl = 0, 					// disable ipu,
		.rgb_ctrl = 0,
		.bgcolor = 0x000000,			// set background color Black
		.colorkey0 = 0,					// disable colorkey
		.colorkey1 = 0,					// disable colorkey
		.alpha = 0xa0,					// alpha value
		.ipu_restart = 0x8000085d,	// ipu restart
		.fg_change = FG_CHANGE_ALL,	// change all initially
		.fg0 = {32, 0, 0, 640, 480},	// bpp, x, y, w, h
		.fg1 = {32, 0, 0, 640, 480},	// bpp, x, y, w, h
	},
};
struct jz4760lcd_info jz4760_info_hdmi_576p = {
	.panel = {
		.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
			   LCD_CFG_NEWDES | LCD_CFG_RECOVER |
			   LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_BST_32,
		//  width,height,freq,hsync,vsync,elw,blw,efw,bfw
		720, 576, 50, 64, 5, 68, 12, 40, 4,
		//800,600,58,128,4,88,40,23,1,i
		//1024,768,60,136,6,160,24,29,3,
	},
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN |		// Use OSD mode
				   LCD_OSDC_ALPHAEN |	// enable alpha
				   LCD_OSDC_F0EN,		// enable Foreground0
				   // LCD_OSDC_F1EN,	// enable Foreground1
		.osd_ctrl = 0,					// disable ipu,
		.rgb_ctrl = 0,
		.bgcolor = 0x000000,			// set background color Black
		.colorkey0 = 0,					// disable colorkey
		.colorkey1 = 0,					// disable colorkey
		.alpha = 0xa0,					// alpha value
		.ipu_restart = 0x8000085d,		// ipu restart
		.fg_change = FG_CHANGE_ALL,		// change all initially
		.fg0 = {32, 0, 0, 720, 576},	// bpp, x, y, w, h
		.fg1 = {32, 0, 0, 720, 576},	// bpp, x, y, w, h
	},
};
struct jz4760lcd_info jz4760_info_hdmi_720p50 = {
	.panel = {
		.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
			   LCD_CFG_NEWDES | LCD_CFG_RECOVER |
			   LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_BST_32,
		//  width,height,freq,hsync,vsync,elw,blw,efw,bfw
		1280, 720, 50, 40, 5, 440, 220, 20, 5,
	},
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN |		// Use OSD mode
				   LCD_OSDC_ALPHAEN |	// enable alpha
				   LCD_OSDC_F0EN,		// enable Foreground0
				   // LCD_OSDC_F1EN,	// enable Foreground1
		.osd_ctrl = 0, 					// disable ipu,
		.rgb_ctrl = 0,
		.bgcolor = 0x000000,			// set background color Black
		.colorkey0 = 0,					// disable colorkey
		.colorkey1 = 0,					// disable colorkey
		.alpha = 0xa0,					// alpha value
		.ipu_restart = 0x8000085d,		// ipu restart
		.fg_change = FG_CHANGE_ALL,		// change all initially
		.fg0 = {32, 0, 0, 1280, 720},	// bpp, x, y, w, h
		.fg1 = {32, 0, 0, 1280, 720},	// bpp, x, y, w, h
	},
};
struct jz4760lcd_info jz4760_info_hdmi_720p60 = {
	.panel = {
		.cfg = LCD_CFG_MODE_GENERIC_TFT | LCD_CFG_MODE_TFT_24BIT |
			   LCD_CFG_NEWDES | LCD_CFG_RECOVER |
			   LCD_CFG_PCP | LCD_CFG_HSP | LCD_CFG_VSP,
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_BST_32,
		//  width,height,freq,hsync,vsync,elw,blw,efw,bfw
		1280, 720, 60, 40, 5, 110, 220, 20, 5,
	},
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN |		// Use OSD mode
				   LCD_OSDC_ALPHAEN |	// enable alpha
				   LCD_OSDC_F0EN,		// enable Foreground0
				   // LCD_OSDC_F1EN,	// enable Foreground1
		.osd_ctrl = 0,					// disable ipu,
		.rgb_ctrl = 0,
		.bgcolor = 0x000000,			// set background color Black
		.colorkey0 = 0,					// disable colorkey
		.colorkey1 = 0,					// disable colorkey
		.alpha = 0xa0,					// alpha value
		.ipu_restart = 0x8000085d,		// ipu restart
		.fg_change = FG_CHANGE_ALL,		// change all initially
		.fg0 = {32, 0, 0, 1280, 720},	// bpp, x, y, w, h
		.fg1 = {32, 0, 0, 1280, 720},	// bpp, x, y, w, h
	},
};

static void set_i2s_external_codec(void)
{
	D("");
#if defined(CONFIG_JZ4760_CYGNUS) || defined(CONFIG_JZ4760B_CYGNUS)
	/* gpio defined based on CYGNUS board */
	__gpio_as_func1(3 * 32 + 12); //blck
	__gpio_as_func0(3 * 32 + 13); //sync
	__gpio_as_func0(4 * 32 + 7);  //sd0
	__gpio_as_func0(4 * 32 + 11); //sd1
	__gpio_as_func0(4 * 32 + 12); //sd2
	__gpio_as_func0(4 * 32 + 13); //sd3
#endif

	__i2s_external_codec();

	__aic_select_i2s();
	__i2s_select_i2s();
	__i2s_as_master();

	REG_AIC_I2SCR |= AIC_I2SCR_ESCLK;

	__i2s_disable_record();
	__i2s_disable_replay();
	__i2s_disable_loopback();

	REG_AIC_FR &= ~AIC_FR_TFTH_MASK;
	REG_AIC_FR |= ((8) << AIC_FR_TFTH_BIT);
	REG_AIC_FR &= ~AIC_FR_RFTH_MASK;
	REG_AIC_FR |= ((8) << AIC_FR_RFTH_BIT);

	__i2s_enable();
}
