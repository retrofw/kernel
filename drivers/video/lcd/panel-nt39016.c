#include <asm/jzsoc.h>

#if defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)/* board pavo */
	#include "../jz4760_lcd.h"
#elif defined(CONFIG_JZ4770_F4770) || defined(CONFIG_JZ4770_PISCES)
	#include "../jz4770_lcd.h"
	#define LCD_RET 	(32*4+2)       /*LCD_DISP_N use for lcd reset*/
	#define LCD_VCC 	(32*4+13)
#else
	#error "Define special lcd pins for your platform."
#endif


#define __lcd_special_pin_init() \
do { \
	__gpio_as_output(LCD_VCC); \
	__gpio_set_pin(LCD_VCC);\
	__gpio_as_output(LCD_RET);\
	udelay(50);\
	__gpio_clear_pin(LCD_RET);\
	mdelay(150);\
	__gpio_set_pin(LCD_RET);\
} while (0)


#define __lcd_special_on() \
do { \
	; \
} while (0)

#define __lcd_special_off() \
do { \
	;\
} while (0)

#define __lcd_display_pin_init() \
do { \
	__gpio_as_output(GPIO_LCD_VCC_EN);	 \
	__gpio_as_output(GPIO_LCD_PWM);	 \
	__lcd_special_pin_init();	   \
} while (0)

#define __lcd_display_on() \
do { \
	__gpio_as_output1(GPIO_LCD_VCC_EN);	\
	__lcd_special_on();			\
	__gpio_as_output(LCD_RET);              \
	udelay(50);\
	__gpio_clear_pin(LCD_RET);              \
	mdelay(550);\
	__gpio_set_pin(LCD_RET);                \
} while (0)

#define __lcd_display_off() \
do { \
	__gpio_clear_pin(GPIO_LCD_VCC_EN);	\
	__lcd_close_backlight();	   \
	__lcd_special_off();	 \
} while (0)

struct jz4760lcd_info jz4760_lcd_panel = {
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER | /* Underrun recover */
			   LCD_CFG_NEWDES | /* 8words descriptor */
			   LCD_CFG_MODE_GENERIC_TFT | /* Serial TFT panel */
			   LCD_CFG_MODE_TFT_24BIT | 	/* output 24bpp */
			   LCD_CFG_HSP | 	/* Hsync polarity: active low */
			   LCD_CFG_VSP,	/* Vsync polarity: leading edge is falling edge */
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,		// 16words burst, enable out FIFO underrun irq
		320, 240, 60, 16, 6, 20, 60, 2, 8,
	 },
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN |			// Use OSD mode
				   // LCD_OSDC_ALPHAEN |	// enable alpha
				   LCD_OSDC_F0EN |			// enable Foreground0
				   LCD_OSDC_F1EN,			// enable Foreground1
		.osd_ctrl = LCD_OSDCTRL_IPU | LCD_OSDCTRL_OSDBPP_15_16,	// enable ipu
		.rgb_ctrl = 0,
		.bgcolor = 0x000000,				// set background color Black
		.colorkey0 = 0x80000000,			// disable colorkey
		.colorkey1 = 0x80000000,			// disable colorkey
		.alpha = 0xA0,						// alpha value
		.ipu_restart = 0x80001000,			// ipu restart
		.fg_change = FG_CHANGE_ALL,			// change all initially
		.fg0 = {16, 0, 0, 320, 240},		// bpp, x, y, w, h
		.fg1 = {16, 0, 0, 320, 240},		// bpp, x, y, w, h
	 },
};
