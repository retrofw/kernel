#ifndef _LINUX_SERIAL_H
#define _LINUX_SERIAL_H

#ifdef __KERNEL__

#define AK5358_MODE_MASTER 0
#define AK5358_MODE_SLAVE  1
#define AK5358_INTERFACE_I2S 0
#define AK5358_INTERFACE_MSB 1

#define BIT_CLK (32 * 3 + 12)  //PD12
#define LR_CLK (32 * 3 + 13)   //PD13
#define I2S_OUT (32 * 4 + 7)   //PE7

struct gpio {
	int num;
	int irq;
	int active_level;
};

struct ak5358_platform_data {
        int fs;
	struct gpio power_pin;
	struct gpio linein_pin;
};

struct ak5358 {
        struct device *dev;
        struct input_dev *input;
        int fs;
        int linein_last;
        int replay_last;

        int fs_last;
        int channel_last;
        int format_last;
        struct gpio power_pin;
        struct gpio linein_pin;
        struct delayed_work work;
        struct mutex lock;
        struct semaphore is_playing;
};

extern int ak5358_register_state_notifier(struct notifier_block *nb);
extern void ak5358_unregister_state_notifier(struct notifier_block *nb);

#endif

#endif
