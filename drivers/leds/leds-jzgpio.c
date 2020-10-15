#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <asm/mach-jz4760b/jz4760b.h>

struct gpio_led_data {
	struct led_classdev cdev;
	unsigned gpio;
	u8 active_low;
};

static void gpio_led_set(struct led_classdev *led_cdev,
                         enum led_brightness value)
{
        struct gpio_led_data *led_dat =
		container_of(led_cdev, struct gpio_led_data, cdev);
	int level;

	if (value == LED_OFF)
		level = 0;
	else
		level = 1;

	if (led_dat->active_low)
		level = !level;

        if (level)
                __gpio_set_pin(led_dat->gpio);
        else
                __gpio_clear_pin(led_dat->gpio);
}

static int __devinit create_gpio_led(const struct gpio_led *template,
                                     struct gpio_led_data *led_dat, struct device *parent)
{
	int ret, state;

	led_dat->cdev.name = template->name;
	led_dat->gpio = template->gpio;
	led_dat->active_low = template->active_low;
        led_dat->cdev.default_trigger = template->default_trigger;

	led_dat->cdev.brightness_set = gpio_led_set;
	if (template->default_state == LEDS_GPIO_DEFSTATE_KEEP)
		state = !!gpio_get_value(led_dat->gpio) ^ led_dat->active_low;
	else
		state = (template->default_state == LEDS_GPIO_DEFSTATE_ON);
        __gpio_as_output(led_dat->gpio);
        __gpio_disable_pull(led_dat->gpio);
        if (led_dat->active_low ^ state)
                __gpio_set_pin(led_dat->gpio);
        else
                __gpio_clear_pin(led_dat->gpio);
	led_dat->cdev.brightness = state ? LED_FULL : LED_OFF;
	if (!template->retain_state_suspended)
		led_dat->cdev.flags |= LED_CORE_SUSPENDRESUME;

	ret = led_classdev_register(parent, &led_dat->cdev);
	if (ret < 0)
		goto err;

	return 0;
err:
	return ret;
}

static void delete_gpio_led(struct gpio_led_data *led)
{
	led_classdev_unregister(&led->cdev);
}

static int jz_leds_gpio_probe(struct platform_device *pdev)
{
        struct gpio_led_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_led_data *leds_data;
	int i, ret = 0;

	if (!pdata)
		return -EBUSY;

	leds_data = kzalloc(sizeof(struct gpio_led_data) * pdata->num_leds,
				GFP_KERNEL);
	if (!leds_data)
		return -ENOMEM;

	for (i = 0; i < pdata->num_leds; i++) {
		ret = create_gpio_led(&pdata->leds[i], &leds_data[i],
				      &pdev->dev);
		if (ret < 0)
			goto err;
	}

	platform_set_drvdata(pdev, leds_data);

	return 0;

err:
	for (i = i - 1; i >= 0; i--)
		delete_gpio_led(&leds_data[i]);

	kfree(leds_data);

	return ret;
}

static int __devexit jz_leds_gpio_remove(struct platform_device *pdev)
{
	int i;
	struct gpio_led_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_led_data *leds_data;

	leds_data = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->num_leds; i++)
		delete_gpio_led(&leds_data[i]);

	kfree(leds_data);

        return 0;
}

#ifdef CONFIG_PM
static int jz_leds_gpio_suspend(struct platform_device *pdev, pm_message_t state)
{
        return 0;
}

static int jz_leds_gpio_resume(struct platform_device *pdev)
{
        return 0;
}
#endif

static struct platform_driver jz_leds_gpio_driver = {
	.probe		= jz_leds_gpio_probe,
	.remove		= jz_leds_gpio_remove,
#ifdef CONFIG_PM
	.suspend	= jz_leds_gpio_suspend,
	.resume		= jz_leds_gpio_resume,
#endif
	.driver		= {
		.name	= "jz-leds-gpio",
		.owner =  THIS_MODULE,
	},
};

static int __init jz_leds_gpio_init(void)
{
	return platform_driver_register(&jz_leds_gpio_driver);
}

static void __exit jz_leds_gpio_exit(void)
{
	platform_driver_unregister(&jz_leds_gpio_driver);
}

module_init(jz_leds_gpio_init);
module_exit(jz_leds_gpio_exit);

MODULE_ALIAS("platform:jz-leds-gpio");
MODULE_AUTHOR("Mingli Feng <mlfeng@ingenic.cn>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("JZ LEDS GPIO Driver");

