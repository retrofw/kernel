#include <asm/jzsoc.h>

#if defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)/* board pavo */
	#include "../jz4760_lcd.h"
	#define SPEN		(32*4+0)   /* LCD_CS */
	#define SPCK		(32*3+11)  /* LCD_SCL */
	#define SPDA		(32*4+2)   /* LCD_SDA */
	#define LCD_RET 	(32*4+4)
// #elif defined(CONFIG_JZ4770_F4770) || defined(CONFIG_JZ4770_PISCES)
	// #include "../jz4770_lcd.h"
#else
	#error "driver/video/jz4760_lcd, please define SPI pins on your board."
#endif


#define __spi_writ_bit16(reg, val) \
do { \
	unsigned char no; \
	unsigned short value; \
	value = ((reg << 8) | (val & 0xFF)); \
	__gpio_set_pin(SPEN); \
	__gpio_clear_pin(SPCK); \
	__gpio_clear_pin(SPDA); \
	__gpio_clear_pin(SPEN); \
	udelay(50); \
	for (no = 0; no < 16; no++) { \
		__gpio_clear_pin(SPCK); \
		if ((value & 0x8000) == 0x8000) \
			__gpio_set_pin(SPDA); \
		else \
			__gpio_clear_pin(SPDA); \
		udelay(50); \
		__gpio_set_pin(SPCK); \
		value <<= 1; \
		udelay(50); \
	 } \
	__gpio_set_pin(SPEN); \
	udelay(50); \
} while (0)

#define __spi_send_value(reg, val) \
do { \
	unsigned char no; \
	unsigned short value; \
	unsigned char cmd_dat = 0; \
	cmd_dat = reg; \
	value = val; \
	__gpio_set_pin(SPEN); \
	__gpio_clear_pin(SPCK); \
	__gpio_clear_pin(SPDA); \
	__gpio_clear_pin(SPEN); \
	udelay(50); \
	if (cmd_dat) \
		value |= 1 << 8; \
	else \
		value &= ~(1 << 8); \
	for (no = 0; no < 9; no++) { \
		__gpio_clear_pin(SPCK); \
		if ((value & 0x100) == 0x100) \
			__gpio_set_pin(SPDA); \
		else\
			__gpio_clear_pin(SPDA); \
		udelay(50); \
		__gpio_set_pin(SPCK); \
		value <<= 1; \
		udelay(50); \
	 } \
	__gpio_set_pin(SPEN); \
	udelay(50); \
} while (0)

#define spi_send_cmd(cmd)     __spi_send_value(0, cmd)
#define spi_send_data(data)   __spi_send_value(1, data)

#define __lcd_special_pin_init() \
do { \
	__gpio_as_output(SPEN); /* use SPDA */ \
	__gpio_as_output(SPCK); /* use SPCK */ \
	__gpio_as_output(SPDA); /* use SPDA */ \
	__gpio_as_output(LCD_RET); \
	udelay(50); \
	__gpio_clear_pin(LCD_RET); \
	mdelay(100); \
	__gpio_set_pin(LCD_RET); \
} while (0)

#define __lcd_special_on() \
do { \
	printk("\n RG IPS\n"); \
	__spi_writ_bit16(0x02, 0x7f); \
	__spi_writ_bit16(0x03, 0x0A); \
	__spi_writ_bit16(0x04, 0x80); \
	__spi_writ_bit16(0x06, 0x90); \
	__spi_writ_bit16(0x08, 0x28); \
	__spi_writ_bit16(0x09, 0x20); \
	__spi_writ_bit16(0x0a, 0x20); \
	__spi_writ_bit16(0x0c, 0x10); \
	__spi_writ_bit16(0x0d, 0x10); \
	__spi_writ_bit16(0x0e, 0x10); \
	__spi_writ_bit16(0x10, 0x7F); \
	__spi_writ_bit16(0x11, 0x3F); \
} while (0)

#define __lcd_special_off() \
do { \
	__gpio_as_output(GPIO_LCD_PWM); \
	__gpio_clear_pin(GPIO_LCD_PWM); \
} while (0)


struct jz4760lcd_info jz4760_lcd_panel = {
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER |	// Underrun recover
			   LCD_CFG_NEWDES |							// 8words descriptor
			   LCD_CFG_MODE_SERIAL_TFT |				// LCD_CFG_MODE_SERIAL_TFT
			   LCD_CFG_HSP | LCD_CFG_PCP |
			   LCD_CFG_MODE_TFT_16BIT,
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,		// 16words burst, enable out FIFO underrun irq
	/* width,height,freq,hsync,vsync,elw,blw,efw,bfw */
		320, 480, 59, 28, 1, 25, 100, 16, 36,
	},
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN |			// Use OSD mode
				   // LCD_OSDC_ALPHAEN |	// enable alpha
				   LCD_OSDC_F0EN |			// enable Foreground0
				   LCD_OSDC_F1EN,			// enable Foreground1
		.osd_ctrl = LCD_OSDCTRL_IPU |		// enable ipu
				    LCD_OSDCTRL_OSDBPP_15_16,
		.rgb_ctrl = LCD_RGBC_ODD_GBR << LCD_RGBC_ODDRGB_BIT,
		.bgcolor = 0x000000,				// set background color Black
		.colorkey0 = 0x80000000,			// disable colorkey
		.colorkey1 = 0x80000000,			// disable colorkey
		.alpha = 0xa0,						// alpha value
		.ipu_restart = 0x80001000,			// ipu restart
		.fg_change = FG_CHANGE_ALL,			// change all initially
		.fg0 = {16, 0, 0, 320, 240},		// bpp, x, y, w, h
		.fg1 = {16, 0, 0, 320, 240},		// bpp, x, y, w, h
	},
};
