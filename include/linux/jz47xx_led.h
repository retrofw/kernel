#ifndef _JZ47XX_LED_H_
#define _JZ47XX_LED_H_

#include <linux/leds.h>

#define LED_DEBUG 0

#if LED_DEBUG
#define MY_DBG(sss, aaa...)											\
	do {															\
		printk("%03d DEBUG %s, " sss, __LINE__, __PRETTY_FUNCTION__, ##aaa);	\
	} while (0)
#define ENTER() printk("%d %s ENTER\n", __LINE__, __FUNCTION__)
#define LEAVE() printk("%d %s LEAVE\n", __LINE__, __FUNCTION__)
#else
#define MY_DBG(sss, aaa...)						\
	do {								\
	} while (0)
#define ENTER()
#define LEAVE()
#endif

struct jz47xx_led_ops {
	void (*lcd_init_backlight) (int level);
	void (*lcd_set_backlight_level) (int level);
	void (*lcd_close_backlight) (void);
};

#endif
