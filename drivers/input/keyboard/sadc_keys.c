#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/mfd/core.h>

#include <asm/jzsoc.h>
#include <linux/input/sadc_keys.h>
#include <linux/mfd/jz4750l-sadc.h>

struct keys_data {
	struct sadc_keys_info *info;
};

struct sadc_keys_drvdata {
	struct input_dev *input;
	const struct mfd_cell *cell;
	struct platform_device *pdev;

	unsigned int gpio_irq;
	unsigned int gpio_state;
	struct timer_list timer;
	struct work_struct work;

	unsigned int sadc_irq;
	struct resource *mem;
	void __iomem *io_base;
	struct completion sadc_completion;

	unsigned int last_code;

	struct sadc_keys_devdata *dev_data;
};

static int sadc_keys_open(struct input_dev *input)
{
	return 0;
}

static void sadc_keys_close(struct input_dev *input)
{
}

static irqreturn_t sadc_irq_handler(int irq, void *dev_id)
{
	struct sadc_keys_drvdata *drv_data = dev_id;

	complete(&drv_data->sadc_completion);

	return IRQ_HANDLED;
}

static unsigned int voltage2code(struct sadc_keys_drvdata *drv_data, unsigned int voltage)
{
	unsigned int num = drv_data->dev_data->info_num, i;
	struct sadc_keys_info *info = drv_data->dev_data->info;

	for (i = 0; i < num; i++) {
		if ((voltage <= info[i].voltage + info[i].fuzz) &&
			(voltage >= info[i].voltage - info[i].fuzz))
			break;
	}

	if (i < num)
		return info[i].code;
	else
		return KEY_RESERVED;
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct sadc_keys_drvdata *drv_data = dev_id;

	disable_irq_nosync(irq);
	mod_timer(&drv_data->timer, jiffies + msecs_to_jiffies(drv_data->dev_data->gpio->debounce_interval));

	return IRQ_HANDLED;
}

static void gpio_timer_handler(unsigned long data)
{
	struct sadc_keys_drvdata *drv_data = (struct sadc_keys_drvdata *)data;

	schedule_work(&drv_data->work);
}

static void gpio_work_handler(struct work_struct *work)
{
	struct sadc_keys_drvdata *drv_data = container_of(work, struct sadc_keys_drvdata, work);
	struct sadc_keys_gpio *gpio = drv_data->dev_data->gpio;
	int state = !!__gpio_get_pin(gpio->num);

	if (gpio->enable_level == LOW_ENABLE)
		state = !state;

	if (state == drv_data->gpio_state) {
		dev_err(&drv_data->pdev->dev, "gpio is same state(%s)!\n", state ? "press" : "up");
		goto err;
	}

	if (state) {
		unsigned long r;
		unsigned int voltage, code;
		/* press */
		INIT_COMPLETION(drv_data->sadc_completion);

		drv_data->cell->enable(drv_data->pdev);
		enable_irq(drv_data->sadc_irq);

		r = wait_for_completion_interruptible_timeout(
			&drv_data->sadc_completion, HZ);
		if (r > 0) {
			voltage = readw(drv_data->io_base);
			voltage = voltage * 3300 / 4096;
			disable_irq(drv_data->sadc_irq);
			drv_data->cell->disable(drv_data->pdev);
		} else {
			dev_err(&drv_data->pdev->dev, "get sadcin val timeout!\n");
			disable_irq(drv_data->sadc_irq);
			drv_data->cell->disable(drv_data->pdev);
			goto err;
		}

		code = voltage2code(drv_data, voltage);
		if (code == KEY_RESERVED) {
			dev_err(&drv_data->pdev->dev, "Can't find matched key(%d)!\n", voltage);
			goto err;
		}
		drv_data->last_code = code;
	} else {
		/* up */
	}
	dev_dbg(&drv_data->pdev->dev, "%s %d keys\n", state ? "press" : "up", drv_data->last_code);
	input_report_key(drv_data->input, drv_data->last_code, state);
	input_sync(drv_data->input);

	drv_data->gpio_state = state;
err:
	if (gpio->enable_level == LOW_ENABLE && state)
        __gpio_as_irq_rise_edge(gpio->num);
	else
        __gpio_as_irq_fall_edge(gpio->num);
	enable_irq(drv_data->gpio_irq);
}

static int __devinit sadc_keys_probe(struct platform_device *pdev)
{
	int ret = 0, i;
	struct adc_cell_platform_data *sadcin_pdata = pdev->dev.platform_data;
	struct sadc_keys_devdata *dev_data;
	struct sadc_keys_drvdata *drv_data;
	struct input_dev *input;

	if (sadcin_pdata) {
		dev_info(&pdev->dev, "get adc_cell_platform_data of \'%s\'.\n", sadcin_pdata->name);
	} else {
		dev_err(&pdev->dev, "miss adc_cell_platform_data.\n");
		return -EINVAL;
	}

	dev_data = sadcin_pdata->dev_data;
	if (dev_data == NULL) {
		dev_err(&pdev->dev, "miss dev_data!\n");
		return -EINVAL;
	}

	drv_data = kzalloc(sizeof(struct sadc_keys_drvdata), GFP_KERNEL);
	input = input_allocate_device();
	if (drv_data == NULL || input == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory.\n");
		return -ENOMEM;
	}

	drv_data->input = input;
	drv_data->cell = sadcin_pdata->cell;
	drv_data->pdev = pdev;
	drv_data->dev_data = dev_data;
	if (!drv_data->cell || !dev_data->gpio || !dev_data->info) {
		dev_err(&pdev->dev, "Invalid parameter.\n");
		ret = -EINVAL;
		goto err_free;
	}

	platform_set_drvdata(pdev, drv_data);
	input_set_drvdata(input, drv_data);

	drv_data->gpio_irq = dev_data->gpio->num + IRQ_GPIO_0;
	drv_data->sadc_irq = platform_get_irq(pdev, 0);
	if (drv_data->gpio_irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource defined\n");
		ret = -ENODEV;
		goto err_free;
	}
	drv_data->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (drv_data->mem == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free;
	}
	drv_data->io_base = ioremap_nocache(drv_data->mem->start, resource_size(drv_data->mem));
	if (drv_data->io_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	input->name = sadcin_pdata->dev_name ? :pdev->name;
	input->phys = "sadc-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = &sadc_keys_open;
	input->close = &sadc_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
	if (dev_data->repeat) {
		input->evbit[0] = BIT(EV_KEY) | BIT(EV_SYN) | BIT(EV_REP);
		dev_info(&pdev->dev, "Support Repeat Event\n");
	} else {
		input->evbit[0] = BIT(EV_KEY) | BIT(EV_SYN);
	}

	for (i = 0; i < dev_data->info_num; i++) {
		input_set_capability(input, EV_KEY, dev_data->info[i].code);
	}

	ret = input_register_device(input);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register input device(error: %d)\n", ret);
		goto err_free_io;
	}

	ret = request_irq(drv_data->sadc_irq, sadc_irq_handler, 0,
					  pdev->name, drv_data);
	if (ret) {
		dev_err(&pdev->dev, "Unable to claim sadc_irq %d(error: %d)\n", drv_data->sadc_irq, ret);
		goto err_free_input;
	}
	disable_irq(drv_data->sadc_irq);

	ret = request_irq(drv_data->gpio_irq, gpio_irq_handler, 0,
							   sadcin_pdata->dev_name, drv_data);
	if (ret) {
		dev_err(&pdev->dev, "Unable to claim gpio_irq %d(error: %d)\n", drv_data->gpio_irq, ret);
		goto err_free_irq;
	}
	disable_irq(drv_data->gpio_irq);
	if (dev_data->gpio->enable_level == LOW_ENABLE)
        __gpio_as_irq_fall_edge(dev_data->gpio->num);
	else
        __gpio_as_irq_rise_edge(dev_data->gpio->num);

	init_completion(&drv_data->sadc_completion);
	setup_timer(&drv_data->timer, gpio_timer_handler, (unsigned long)drv_data);
	INIT_WORK(&drv_data->work, gpio_work_handler);

	drv_data->gpio_state = 0;
	schedule_work(&drv_data->work);

	/* device_init_wakeup(&pdev->dev, wakeup); */

	dev_info(&pdev->dev, "sadc keys register successful\n");

	return 0;

err_free_irq:
	free_irq(drv_data->sadc_irq, drv_data);
err_free_input:
	input_unregister_device(drv_data->input);
err_free_io:
	iounmap(drv_data->io_base);
err_free_mem:
	release_mem_region(drv_data->mem->start, resource_size(drv_data->mem));
err_free:
	input_free_device(drv_data->input);
	kfree(drv_data);

	return ret;
}

static int __devexit sadc_keys_remove(struct platform_device *pdev)
{
	struct sadc_keys_drvdata *drv_data = platform_get_drvdata(pdev);

	cancel_work_sync(&drv_data->work);
	del_timer_sync(&drv_data->timer);
	free_irq(drv_data->gpio_irq, drv_data);
	free_irq(drv_data->sadc_irq, drv_data);
	input_unregister_device(drv_data->input);
	iounmap(drv_data->io_base);
	release_mem_region(drv_data->mem->start, resource_size(drv_data->mem));
	input_free_device(drv_data->input);
	kfree(drv_data);

	return 0;
}

static struct platform_driver sadc_keys_driver = {
	.probe = sadc_keys_probe,
	.remove = __devexit_p(sadc_keys_remove),
	.driver = {
		.name = "sadcin",
		.owner = THIS_MODULE,
	},
};

static int __init sadc_keys_init(void)
{
	return platform_driver_register(&sadc_keys_driver);
}

static void __exit sadc_keys_exit(void)
{
	platform_driver_unregister(&sadc_keys_driver);
}

module_init(sadc_keys_init);
module_exit(sadc_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jiang Tao<tao.jiang@ingneic.com");
MODULE_DESCRIPTION("Keyboard driver for Ingenic SADC");
