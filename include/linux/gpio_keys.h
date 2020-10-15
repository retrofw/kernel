#ifndef _GPIO_KEYS_H
#define _GPIO_KEYS_H

struct gpio_keys_button {
	/* Configuration parameters */
	int code;		/* input event code (KEY_*, SW_*) */
	int gpio;
	int active_low;
	char *desc;
	int type;		/* input event type (EV_KEY, EV_SW) */
	int wakeup;		/* configure the button as a wake-up source */
	int debounce_interval;	/* debounce ticks interval in msecs */
};

struct gpio_keys_platform_data {
	struct gpio_keys_button *buttons;
	int nbuttons;
	unsigned int rep:1;		/* enable input subsystem auto repeat */
};

struct rotary_encoder_data {
	int switch_gpio;
	int right_gpio;
	int left_gpio;
	int switch_code;
	int right_code;
	int left_code;
	int switch_active_low; 
	int right_active_low;
	int left_active_low;
	int type;		/* input event type (EV_KEY, EV_SW) */
	char *name;
};
struct jz_rotary_encoder_platform_data {
	struct rotary_encoder_data *buttons;
	int nbuttons;
	unsigned int rep:1;		/* enable input subsystem auto repeat */
};
#endif
