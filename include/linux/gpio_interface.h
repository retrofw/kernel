#ifndef GPIO_INTERFACE_BATTERY_H
#define GPIO_INTERFACE_BATTERY_H

struct gpio_detect_pin {
	int pin;
	int low_active;
};

struct gpio_interface_platform_data {
	struct gpio_detect_pin gpio_usb_dete;
	struct gpio_detect_pin gpio_bat_dete;
	struct gpio_detect_pin gpio_dc_dete;
	struct gpio_detect_pin gpio_charg_stat;
};

#endif
