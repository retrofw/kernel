#include <asm/jzsoc.h>

#if defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)/* board pavo */
	#include "../jz4760_lcd.h"
	#define SPEN		(32*4+0)       /*LCD_CS*/
	#define SPCK		(32*3+11)       /*LCD_SCL*/
	#define SPDA		(32*4+2)       /*LCD_SDA*/
	#define LCD_RET 	(32*4+4)
// #elif defined(CONFIG_JZ4770_F4770) || defined(CONFIG_JZ4770_PISCES)
	// #include "../jz4770_lcd.h"
#else
	#error "driver/video/jz4760_lcd, please define SPI pins on your board."
#endif

#define __spi_send_value(reg, val) \
do { \
	unsigned char no; \
	unsigned short value; \
	unsigned char cmd_dat=0; \
	cmd_dat =reg; \
	value = val; \
	__gpio_set_pin(SPEN); \
	__gpio_clear_pin(SPCK); \
	__gpio_clear_pin(SPDA); \
	__gpio_clear_pin(SPEN); \
	udelay(50); \
	if(cmd_dat) \
		value |= 1<<8; \
	else \
		value &= ~(1<<8); \
	for(no=0;no<9;no++) \
	{\
		__gpio_clear_pin(SPCK); \
		if((value&0x100)==0x100) \
			__gpio_set_pin(SPDA); \
		else\
			__gpio_clear_pin(SPDA); \
		udelay(50); \
		__gpio_set_pin(SPCK); \
		value <<= 1; \
		udelay(50); \
	 }\
	__gpio_set_pin(SPEN); \
	udelay(50); \
} while (0)

#define spi_send_cmd(cmd)     __spi_send_value(0,cmd)
#define spi_send_data(data)   __spi_send_value(1,data)
#if 1 //allen add for rgb lcd or tvout
#define __lcd_special_pin_init() \
 do { \
 }while (0)

#else //tm3.75
#define __lcd_special_pin_init() \
 do { \
 	printk("\n 43CPT80-40 REST ... \n"); \
	__gpio_as_output(SPEN); /* use SPDA */ \
	__gpio_as_output(SPCK); /* use SPCK */ \
	__gpio_as_output(SPDA); /* use SPDA */ \
	__gpio_as_output(LCD_RET); \
	udelay(50); \
	__gpio_clear_pin(LCD_RET); \
	mdelay(150); \
	__gpio_set_pin(LCD_RET); \
 }while (0)
#endif

#if 1 //rgb lcd or tvout
#define __lcd_special_on() \
 do { \
 	printk("\n  rgb or tvout ... \n"); \
\
 } while (0)

#else //TM3.7

#define __lcd_special_on() \
 do { \
	printk("\n tm3.7 lcd ... \n"); \
	 spi_send_cmd(0xB9); \
	 spi_send_data(0xFF); \
	 spi_send_data(0x83); \
	 spi_send_data(0x57); \
	 mdelay(5); \
	 spi_send_cmd(0xB6); /*VCOMDC*/ \
	 spi_send_data(0x47); \
	 spi_send_cmd(0x35); \
\
	spi_send_cmd(0x11); \
	mdelay(150); \
\
	spi_send_cmd(0x3A); \
	spi_send_data(0x66);/*6-18pix, 5-16pix */ \
\
	spi_send_cmd(0xCC); \
	spi_send_data(0x09); \
\
	spi_send_cmd(0xb0); \
	spi_send_data(0x67); /*internal osc >60HZ*/ \
\
	spi_send_cmd(0x36);/*dirction*/ \
	spi_send_data(0x60);/*0x20*/ \
\
	spi_send_cmd(0x2A); \
	spi_send_data(0x00); \
	spi_send_data(0x00); \
	spi_send_data(0x01); \
	spi_send_data(0xdf); \
\
	spi_send_cmd(0x2B); \
	spi_send_data(0x00); \
	spi_send_data(0x00); \
	spi_send_data(0x01); \
	spi_send_data(0x3f); \
\
	spi_send_cmd(0xB3); \
	/*spi_send_data(0x53); //0x43 dirction display*/ \
	spi_send_data(0x02);/*use grame */ \
	spi_send_data(0x0e); \
	spi_send_data(0x02); \
	spi_send_data(0x02); \
\
	spi_send_cmd(0xB1); \
	spi_send_data(0x00); \
	spi_send_data(0x15); \
	spi_send_data(0x1E); \
	spi_send_data(0x1E); \
	spi_send_data(0x83); \
	spi_send_data(0x48); \
\
	spi_send_cmd(0xC0); \
	spi_send_data(0x24); \
	spi_send_data(0x24); \
	spi_send_data(0x01); \
	spi_send_data(0x3C); \
	spi_send_data(0x1E); \
	spi_send_data(0x08); \
\
	spi_send_cmd(0xB4); \
	spi_send_data(0x01); \
	spi_send_data(0x40); \
	spi_send_data(0x00); \
	spi_send_data(0x2A); \
	spi_send_data(0x2A); \
	spi_send_data(0x0D); \
	spi_send_data(0x4F); \
\
	/*set gamma*/ \
	spi_send_cmd(0xE0); \
	spi_send_data(0x02); \
	spi_send_data(0x08); \
	spi_send_data(0x11); \
	spi_send_data(0x23); \
	spi_send_data(0x2C); \
	spi_send_data(0x40); \
	spi_send_data(0x4A); \
	spi_send_data(0x52); \
	spi_send_data(0x48); \
	spi_send_data(0x41); \
	spi_send_data(0x3C); \
	spi_send_data(0x33); \
	spi_send_data(0x2E); \
	spi_send_data(0x28); \
	spi_send_data(0x27); \
	spi_send_data(0x1B); \
	spi_send_data(0x02); \
	spi_send_data(0x08); \
	spi_send_data(0x11); \
	spi_send_data(0x23); \
	spi_send_data(0x2C); \
	spi_send_data(0x40); \
\
	spi_send_data(0x4A); \
	spi_send_data(0x52); \
	spi_send_data(0x48); \
	spi_send_data(0x41); \
	spi_send_data(0x3C); \
	spi_send_data(0x33); \
	spi_send_data(0x2E); \
	spi_send_data(0x28); \
	spi_send_data(0x27); \
	spi_send_data(0x1B); \
	spi_send_data(0x00); \
	spi_send_data(0x01); \
\
	spi_send_cmd(0x29); \
	mdelay(50); \
 } while (0)

#endif

#define __lcd_special_off() \
  do { \
    __gpio_as_output(GPIO_LCD_PWM); \
	__gpio_clear_pin(GPIO_LCD_PWM); \
  } while (0)


struct jz4760lcd_info jz4760_lcd_panel = {
	.panel = {
		.cfg = LCD_CFG_LCDPIN_LCD | LCD_CFG_RECOVER |	// Underrun recover
			   LCD_CFG_NEWDES |
			   LCD_CFG_MODE_GENERIC_TFT |				// General TFT panel
			   LCD_CFG_MODE_TFT_18BIT |					// output 18bpp
			   LCD_CFG_HSP | 							// Hsync polarity: active low
			   LCD_CFG_VSP,
		.slcd_cfg = 0,
		.ctrl = LCD_CTRL_OFUM | LCD_CTRL_BST_16,		// 16words burst, enable out FIFO underrun irq
	/* width,height,freq,hsync,vsync,elw,blw,efw,bfw */
		480, 272, 60, 41, 10, 2, 2, 2, 2,
	},
	.osd = {
		.osd_cfg = LCD_OSDC_OSDEN |			// Use OSD mode
				   // LCD_OSDC_ALPHAEN |	// enable alpha
				   LCD_OSDC_F0EN |			// enable Foreground0
				   LCD_OSDC_F1EN,			// enable Foreground1
		.osd_ctrl = LCD_OSDCTRL_IPU |		// enable ipu
				    LCD_OSDCTRL_OSDBPP_15_16,
		.rgb_ctrl = 0,
		.bgcolor = 0x000000,				// set background color Black
		.colorkey0 = 0x80000000,			// disable colorkey
		.colorkey1 = 0x80000000,			// disable colorkey
		.alpha = 0xa0,						// alpha value
		.ipu_restart = 0x80001000,			// ipu restart
		.fg_change = FG_CHANGE_ALL,			// change all initially
		.fg0 = {16, 0, 0, 480, 272},		// bpp, x, y, w, h
		.fg1 = {16, 0, 0, 480, 272},		// bpp, x, y, w, h
	 },
};
