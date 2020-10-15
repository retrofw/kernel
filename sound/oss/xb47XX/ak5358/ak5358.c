#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/ak5358.h>

#include <asm/mach-jz4760b/jz4760b.h>
#include "ak5358.h"

#ifdef CONFIG_AK5358_DEBUG
static int ak5358_debug = 1;
#define AK5358_DEBUG_MSG(msg...)                                \
        do {							\
		if (unlikely(ak5358_debug)) {                   \
			ak5358_printk("ak5358", msg);                   \
		}                                                       \
	} while(0)

static ssize_t ak5358_debug_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "%d\n", ak5358_debug);
}

static ssize_t ak5358_debug_set(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
        if (strncmp(buf, "1", 1) == 0)
                ak5358_debug = 1;
        else
                ak5358_debug = 0;

        return count;
}

static DEVICE_ATTR(debug, S_IWUSR | S_IRUSR | S_IROTH,
                   ak5358_debug_show, ak5358_debug_set);

static struct attribute *ak5358_attributes[] = {
	&dev_attr_debug.attr,
	NULL,
};

static const struct attribute_group ak5358_attr_group = {
	.attrs = ak5358_attributes,
};
#else
#define AK5358_DEBUG_MSG(msg...)  do {  } while(0)
#endif

struct ak5358 *ak5358_golbal = NULL;

static void ak5358_set_power(struct ak5358 *ak5358, int on)
{
        if (!ak5358->power_pin.active_level)
                on = !on;
        if (on)
                __gpio_set_pin(ak5358->power_pin.num);
        else
                __gpio_clear_pin(ak5358->power_pin.num);
}

static BLOCKING_NOTIFIER_HEAD(ak5358_notify_chain);

int ak5358_register_state_notifier(struct notifier_block *nb)
{
        return blocking_notifier_chain_register(&ak5358_notify_chain, nb);
}
EXPORT_SYMBOL(ak5358_register_state_notifier);

void ak5358_unregister_state_notifier(struct notifier_block *nb)
{
        blocking_notifier_chain_unregister(&ak5358_notify_chain, nb);
}
EXPORT_SYMBOL(ak5358_unregister_state_notifier);

static void ak5358_notifier_call_chain_sync(struct ak5358 *ak5358, int state)
{
	blocking_notifier_call_chain(&ak5358_notify_chain, state, ak5358);
}

static int ak5358_input_init(struct ak5358 *ak5358)
{
        struct input_dev *input = NULL;
	int ret = 0;

        input = input_allocate_device();
        if (input == NULL) {
                ret = -ENOMEM;
                goto out;
        }

        input->name = "ak5358";
        input->dev.parent = ak5358->dev;

        __set_bit(EV_MSC, input->evbit);
        __set_bit(MSC_RAW, input->mscbit);

        if ((ret = input_register_device(input)) < 0)
		goto error;
        ak5358->input = input;
        return 0;

error:
        input_free_device(input);
out:
        return ret;
}

static void ak5358_input_cleanup(struct ak5358 *ak5358)
{
	if (ak5358->input) {
		input_unregister_device(ak5358->input);
                input_free_device(ak5358->input);
                ak5358->input = NULL;
        }
}

static void detect_linein_edge(struct ak5358 *ak5358)
{
        if(gpio_get_value(ak5358->linein_pin.num) == 0) {
                __gpio_as_irq_rise_edge(ak5358->linein_pin.num);
	} else {
		__gpio_as_irq_fall_edge(ak5358->linein_pin.num);
	}
}

static void linein_plug_change(struct ak5358 *ak5358, int in)
{
        mutex_lock(&ak5358->lock);
        AK5358_DEBUG_MSG("%s, in = %d\n", __func__, in);

        ak5358_notifier_call_chain_sync(ak5358, in);
        ak5358_set_power(ak5358, in);

        mutex_unlock(&ak5358->lock);
}

static void linein_detect_work(struct work_struct *work)
{
	struct ak5358 *ak5358;
        int value = 0;
	ak5358 = container_of(work, struct ak5358, work.work);
        value = gpio_get_value(ak5358->linein_pin.num);

        detect_linein_edge(ak5358);

        if (value == ak5358->linein_last)
                goto finish;
        AK5358_DEBUG_MSG("%s, linein pin = %d\n", __func__, value);

        ak5358->linein_last = value;
        linein_plug_change(ak5358, value == ak5358->linein_pin.active_level ? 1 : 0);
        input_event(ak5358->input, EV_MSC, MSC_RAW,
                    value == ak5358->linein_pin.active_level ? 1 : 0);
        input_sync(ak5358->input);
finish:
        enable_irq(ak5358->linein_pin.irq);
}

static irqreturn_t linein_detect_interrupt(int irq, void *dev_id)
{
	struct ak5358 *ak5358 = (struct ak5358 *)dev_id;
	disable_irq_nosync(irq);
	schedule_delayed_work(&ak5358->work, msecs_to_jiffies(300));

	return IRQ_HANDLED;
}

static int ak5358_probe(struct platform_device *pdev)
{
        struct ak5358 *ak5358 = NULL;
        struct ak5358_platform_data *pdata = pdev->dev.platform_data;
        int ret = 0;

        AK5358_DEBUG_MSG("%s\n", __func__);

        ak5358 = kzalloc(sizeof(*ak5358), GFP_KERNEL);
	if (!ak5358) {
		dev_err(&pdev->dev, "No enough memory!\n");
                ret = -ENOMEM;
                goto out;
	}

        platform_set_drvdata(pdev, ak5358);
        ak5358->dev = &pdev->dev;
        INIT_DELAYED_WORK(&ak5358->work, linein_detect_work);
        mutex_init(&ak5358->lock);
        sema_init(&ak5358->is_playing, 1);

        ak5358->fs = pdata->fs;
        ak5358->power_pin.num = pdata->power_pin.num;
        ak5358->power_pin.active_level = pdata->power_pin.active_level;
        ak5358->linein_pin.num = pdata->linein_pin.num;
        ak5358->linein_pin.irq = pdata->linein_pin.irq;
        ak5358->linein_pin.active_level = pdata->linein_pin.active_level;

        AK5358_DEBUG_MSG("power_pin = %d, active = %d\n",
                         ak5358->power_pin.num, ak5358->power_pin.active_level);
        AK5358_DEBUG_MSG("linein_pin = %d, irq = %d, active = %d\n",
                         ak5358->linein_pin.num, ak5358->linein_pin.irq,
                         ak5358->linein_pin.active_level);

        __gpio_as_output(ak5358->power_pin.num);
        ak5358_set_power(ak5358, 0);

        __gpio_mask_irq(ak5358->linein_pin.num);
        __gpio_as_input(ak5358->linein_pin.num);
        __gpio_enable_pull(ak5358->linein_pin.num);
        ret = request_irq(ak5358->linein_pin.irq,
                          linein_detect_interrupt,
                          IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                          "linein-detect", ak5358);
        if (ret) {
                dev_err(&pdev->dev, "request linein-detect irq fail\n");
                goto fail_req_irq;
        } else
                disable_irq(ak5358->linein_pin.irq);

        ret = ak5358_input_init(ak5358);
        if (ret != 0) {
                dev_err(&pdev->dev, "Input init error!\n");
                goto input_init_error;
        }

#ifdef CONFIG_AK5358_DEBUG
        ret = sysfs_create_group(&pdev->dev.kobj, &ak5358_attr_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to create sysfs group!\n");
                goto sysfs_create_error;
	}
#endif
        ak5358_golbal = ak5358;

        return 0;

#ifdef CONFIG_AK5358_DEBUG
sysfs_create_error:
        ak5358_input_cleanup(ak5358);
#endif
input_init_error:
        free_irq(ak5358->linein_pin.irq, ak5358);
fail_req_irq:
        kfree(ak5358);
out:
        return ret;

        return 0;
}

static int ak5358_remove(struct platform_device *pdev)
{
        struct ak5358 *ak5358 = platform_get_drvdata(pdev);

        disable_irq(ak5358->linein_pin.irq);
        cancel_delayed_work(&ak5358->work);
        flush_scheduled_work();
#ifdef CONFIG_AK5358_DEBUG
        sysfs_remove_group(&pdev->dev.kobj, &ak5358_attr_group);
#endif
        ak5358_input_cleanup(ak5358);
        free_irq(ak5358->linein_pin.irq, ak5358);
        kfree(ak5358);

        return 0;
}

#ifdef CONFIG_PM
static int ak5358_suspend(struct platform_device *pdev, pm_message_t state)
{
        struct ak5358 *ak5358 = platform_get_drvdata(pdev);
        enable_irq_wake(ak5358->linein_pin.irq);

        if (ak5358->linein_last == ak5358->linein_pin.active_level)
             ak5358_set_power(ak5358, 0);

        return 0;
}

static int ak5358_resume(struct platform_device *pdev)
{
        struct ak5358 *ak5358 = platform_get_drvdata(pdev);
        disable_irq_wake(ak5358->linein_pin.irq);

        if (ak5358->linein_last == ak5358->linein_pin.active_level)
                ak5358_set_power(ak5358, 1);

        return 0;
}
#endif

static struct platform_driver ak5358_driver = {
	.probe		= ak5358_probe,
	.remove		= ak5358_remove,
#ifdef CONFIG_PM
	.suspend	= ak5358_suspend,
	.resume		= ak5358_resume,
#endif
	.driver		= {
		.name	= "ak5358",
		.owner =  THIS_MODULE,
	},
};

static int __init ak5358_init(void)
{
	return platform_driver_register(&ak5358_driver);
}

static void __exit ak5358_exit(void)
{
	platform_driver_unregister(&ak5358_driver);
}

static int __init ak5358_late_init(void)
{
        struct ak5358 * ak5358 = ak5358_golbal;

        __gpio_ack_irq(ak5358->linein_pin.num);
        ak5358->linein_last = gpio_get_value(ak5358->linein_pin.num);
        if (ak5358->linein_last == ak5358->linein_pin.active_level ? 1 : 0)
                linein_plug_change(ak5358, 1);
        detect_linein_edge(ak5358);
        __gpio_unmask_irq(ak5358->linein_pin.num);
        enable_irq(ak5358->linein_pin.irq);

        return 0;
}

late_initcall(ak5358_late_init);

module_init(ak5358_init);
module_exit(ak5358_exit);

MODULE_ALIAS("ak5358: audio linein");
MODULE_AUTHOR("Mingli Feng <mlfeng@ingenic.cn>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ak5358: audio linein");
