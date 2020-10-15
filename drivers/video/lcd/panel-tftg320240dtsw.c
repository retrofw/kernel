#include <asm/jzsoc.h>

#define LCD_RESET_PIN	(32*5+10)// LCD_REV, GPF10


#define __lcd_special_on() \
do { \
	__gpio_as_output(32*3+30); \
	__gpio_clear_pin(32*3+30); \
	__gpio_as_output(LCD_RESET_PIN); \
	__gpio_set_pin(LCD_RESET_PIN); \
	udelay(100); \
	__gpio_clear_pin(LCD_RESET_PIN); \
	udelay(100); \
	__gpio_set_pin(LCD_RESET_PIN); \
} while (0)
