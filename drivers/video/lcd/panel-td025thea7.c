#include <asm/jzsoc.h>

#if defined(CONFIG_JZ4760_LCD_TOPPOLY_TD025THEA7_RGB_DELTA)
	#define PANEL_MODE 0x02		/* RGB Delta */
#elif defined(CONFIG_JZ4760_LCD_TOPPOLY_TD025THEA7_RGB_DUMMY)
	#define PANEL_MODE 0x00		/* RGB Dummy */
#endif

#if defined(CONFIG_JZ4760_LEPUS) || defined(CONFIG_JZ4760B_LEPUS)/* board LEPUS */
	#include "../jz4760_lcd.h"
	#define SPEN	(32*2+29)       //GPB29
	#define SPCK	(32*2+28)       //GPB28
	#define SPDA	(32*2+21)       //GPB21
	#define LCD_RET (32*5+6)        // GPF6  //use for lcd reset
#elif defined(CONFIG_JZ4770_F4770) || defined(CONFIG_JZ4770_PISCES)
	#include "../jz4770_lcd.h"
	#define SPEN	(32*2+29)       //GPB29
	#define SPCK	(32*2+28)       //GPB28
	#define SPDA	(32*2+21)       //GPB21
	#define LCD_RET (32*5+6)        // GPF6  //use for lcd reset
#else
	#error "please define SPI pins on your board."
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
		udelay(100); \
		__gpio_clear_pin(SPCK); \
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
		__gpio_clear_pin(SPCK); \
		__gpio_set_pin(SPEN); \
		udelay(100); \
	} while (0)

#define __spi_write_reg(reg, val) \
	do {\
		__spi_write_reg1((reg<<2), val); \
		udelay(100); \
	}while(0)

#define __lcd_special_pin_init() \
	do { \
		__gpio_as_output(SPEN); /* use SPDA */ \
		__gpio_as_output(SPCK); /* use SPCK */ \
		__gpio_as_output(SPDA); /* use SPDA */ \
		__gpio_as_output(SPDA); /* use reset */ \
		__gpio_as_output(LCD_RET); /* use reset */ \
		__gpio_set_pin(LCD_RET); \
		mdelay(15); \
		__gpio_clear_pin(LCD_RET); \
		mdelay(15); \
		__gpio_set_pin(LCD_RET); \
	} while (0)

#define __lcd_special_on() \
	do { \
	mdelay(10); \
	__spi_write_reg(0x00, 0x10); \
	__spi_write_reg(0x01, 0xB1); \
	__spi_write_reg(0x00, 0x10); \
	__spi_write_reg(0x01, 0xB1); \
	__spi_write_reg(0x02, PANEL_MODE); /* RGBD MODE */ \
	__spi_write_reg(0x03, 0x01); /* Noninterlace*/ \
	mdelay(10); \
	} while (0)

#define __lcd_special_off() \
	do { \
	} while (0)
