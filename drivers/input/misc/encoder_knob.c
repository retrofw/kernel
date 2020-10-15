/*
 * linux/drivers/input/misc/encoder_knob.c
 *
 * Encoder knob is a encode device which always be use for change volume.
 *
 * User applications can access to this device via /dev/input/eventX
 *
 * Copyright (c) 2005 - 2009  Ingenic Semiconductor Inc.
 *
 * Author: Andy <tjin@ingenic.cn>
 *     
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/rtc.h>
#include <linux/gpio_keys.h>

#include <asm/gpio.h>
#include <asm/jzsoc.h>

//#define DEBUG 1
#ifdef DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...)
#endif

static char *old_state = NULL;

static void enable_gpio_irqs(struct jz_rotary_encoder_platform_data *pdata)
{
	int i;
	struct rotary_encoder_data *button;

	for (i = 0; i < pdata->nbuttons; i++) {
		button = &pdata->buttons[i];

		if (button->switch_active_low == 1)
			__gpio_as_irq_fall_edge(button->switch_gpio);
		else
			__gpio_as_irq_rise_edge(button->switch_gpio);
		
		if (button->right_active_low == 1)
			__gpio_as_irq_fall_edge(button->right_gpio);
		else
			__gpio_as_irq_rise_edge(button->right_gpio);
		
		if (button->left_active_low == 1)
			__gpio_as_irq_fall_edge(button->left_gpio);
		else
			__gpio_as_irq_rise_edge(button->left_gpio);
	}
}

static irqreturn_t select_key_interrupt(int irq, void *dev_id)
{
	int state,i;
	int gpio = -1;
	struct rotary_encoder_data *button= NULL;	
	struct platform_device *pdev = dev_id;
	struct jz_rotary_encoder_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input = platform_get_drvdata(pdev);
	
	for (i = 0; i < pdata->nbuttons; i++) {
		button = &pdata->buttons[i];
		if (irq == (button->switch_gpio + IRQ_GPIO_0)){
			gpio = button->switch_gpio;
			break;	
		}
	}
	
	if (gpio == -1){
		printk("encoder select key irq failed.\n");
		return IRQ_HANDLED;
	}
	
	state = __gpio_get_pin(button->switch_gpio);
	/* The time is confirmed by test */
	mdelay(2);
	state = __gpio_get_pin(button->switch_gpio);

	if(state)
		__gpio_as_irq_fall_edge(button->switch_gpio);
	else
		__gpio_as_irq_rise_edge(button->switch_gpio);

	if(state ^ button->switch_active_low){
		input_report_key(input, button->switch_code, 1);
		dprintk("key down\n");
	}else{
		input_report_key(input, button->switch_code, 0);
		dprintk("key up\n");
	}
	input_sync(input);

	return IRQ_HANDLED;
}

static irqreturn_t encoder_interrupt(int irq, void *dev_id)
{
	int i, state;
	int gpio = -1;
	struct rotary_encoder_data *button = NULL;
	char new_state[2] = {-1, -1};
	struct platform_device *pdev = dev_id;
	struct jz_rotary_encoder_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input = platform_get_drvdata(pdev);

	for (i = 0; i < pdata->nbuttons; i++) {

		button = &pdata->buttons[i];
		if (irq == (button->right_gpio + IRQ_GPIO_0)){
			gpio = button->right_gpio;
		}else if(irq == (button->left_gpio + IRQ_GPIO_0)){
			gpio = button->left_gpio;
		}else
			continue;
		
		/* This time is very important. If it's too longer, audio will underrun. If it's too shorter, encoder will not filter the wave when bad contact */
		udelay(50);                             //This is important.
		state = __gpio_get_pin(gpio);
		if(state)
			__gpio_as_irq_fall_edge(gpio);
		else
			__gpio_as_irq_rise_edge(gpio);
		break;
	}
	
	if (gpio == -1){
		printk("encoder irq failed.\n");
		return IRQ_HANDLED;
	}
	mdelay(1);                                        //This is important.
	new_state[0] = __gpio_get_pin(button->right_gpio);
	new_state[1] = __gpio_get_pin(button->left_gpio);
	
	/* Here is not use interrupt bottom half method, you can add it */
	if((new_state[0] == 0) && (new_state[1] == 0)){         
		if((old_state[i] == 1)&&(old_state[i+1] == 0)){
			input_report_key(input, button->right_code, 0);
			input_report_key(input, button->right_code, 1);
			input_sync(input);
			dprintk("u\n");
		}else if((old_state[i] == 0)&&(old_state[i+1] == 1)){
			input_report_key(input, button->left_code, 0);
			input_report_key(input, button->left_code, 1);
			input_sync(input);
			dprintk("d\n");
		}else{
		}	
	}

	old_state[i] = new_state[0];
	old_state[i+1] = new_state[1];
	return IRQ_HANDLED;
}

static int __devinit encoder_probe(struct platform_device *pdev)
{
	struct jz_rotary_encoder_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input;
	int i, error, irq;
	unsigned int type;
	struct rotary_encoder_data *button;
	
	old_state = kzalloc(sizeof(short) * pdata->nbuttons, GFP_KERNEL);
	if (old_state == NULL) {
		printk(KERN_ERR "encoder probe: out of memory.\n");
		return -ENOMEM;
	}

	input = input_allocate_device();
	if (!input){
		kfree(old_state);
		return -ENOMEM;	
	}

	if (pdata->nbuttons < 1) {
		printk("%s %d no key register in!\n",__FUNCTION__, __LINE__);
		kfree(old_state);
		return -EINVAL;
	}

	platform_set_drvdata(pdev, input);

	input->name = pdev->name;
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	/*
	input->evbit[0] = BIT(EV_KEY) | BIT(EV_SYN);
	set_bit(KEY_VOLUMEUP, input->keybit);
	set_bit(KEY_VOLUMEDOWN, input->keybit);
	set_bit(KEY_SELECT, input->keybit);
	*/
	for (i = 0; i < pdata->nbuttons; i++) {
		button = &pdata->buttons[i];
		type = button->type ?: EV_KEY;

		if (button->switch_gpio != -1){
			irq = IRQ_GPIO_0 + button->switch_gpio;
			if (irq < 0) {
				error = irq;
				pr_err("%s: Unable to get irq number"
						" for GPIO %d, error %d\n", pdev->name,
						button->switch_gpio, error);
				goto fail;
			}
			error = request_irq(irq, select_key_interrupt,
					IRQF_SAMPLE_RANDOM | IRQF_DISABLED,
					button->name ? button->name : "encoder_switch",
					pdev);
			if (error) {
				pr_err("%s: Unable to claim irq %d; error %d\n",
						pdev->name, irq, error);
				goto fail;
			}
			input_set_capability(input, type, button->switch_code);
		}
		if (button->right_gpio != -1){
			irq = IRQ_GPIO_0 + button->right_gpio;
			if (irq < 0) {
				error = irq;
				pr_err("%s: Unable to get irq number"
						" for GPIO %d, error %d\n", pdev->name,
						button->right_gpio, error);
				goto fail;
			}
			error = request_irq(irq, encoder_interrupt,
				    IRQF_SAMPLE_RANDOM | IRQF_DISABLED,
				    button->name ? button->name : "encoder_right",
				    pdev);
			if (error) {
				pr_err("%s: Unable to claim irq %d; error %d\n",
						pdev->name, irq, error);
				goto fail;
			}
			input_set_capability(input, type, button->right_code);
		}
		if (button->left_gpio != -1){
			irq = IRQ_GPIO_0 + button->left_gpio;
			if (irq < 0) {
				error = irq;
				pr_err("%s: Unable to get irq number"
						" for GPIO %d, error %d\n", pdev->name,
						button->left_gpio, error);
				goto fail;
			}
			error = request_irq(irq, encoder_interrupt,
				    IRQF_SAMPLE_RANDOM | IRQF_DISABLED,
				    button->name ? button->name : "encoder_left",
				    pdev);
			if (error) {
				pr_err("%s: Unable to claim irq %d; error %d\n",
						pdev->name, irq, error);
				goto fail;
			}
			input_set_capability(input, type, button->left_code);
		}
	}

	/* Enable all GPIO irqs */
	enable_gpio_irqs(pdata);

	error = input_register_device(input);
	if (error) {
		pr_err("%s: Unable to register input device, "
		       "error: %d\n", pdev->name, error);
		goto fail;
	}

	return 0;

 fail:
#if 0
	while (--i >= 0) {
		free_irq(pdata->buttons[i].gpio + IRQ_GPIO_0 , pdev);
	}
#endif
	platform_set_drvdata(pdev, NULL);
	input_free_device(input);
	kfree(old_state);

	return error;
}

static int __devexit encoder_remove(struct platform_device *pdev)
{
	struct jz_rotary_encoder_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input = platform_get_drvdata(pdev);
	int i;
	int irq;

	for (i = 0; i < pdata->nbuttons; i++) {
		if (pdata->buttons[i].switch_gpio != -1){
			irq = pdata->buttons[i].switch_gpio + IRQ_GPIO_0;
			free_irq(irq, pdev);
		}
		if (pdata->buttons[i].right_gpio != -1){
			irq = pdata->buttons[i].right_gpio + IRQ_GPIO_0;
			free_irq(irq, pdev);
		}
		if (pdata->buttons[i].left_gpio != -1){
			irq = pdata->buttons[i].left_gpio + IRQ_GPIO_0;
			free_irq(irq, pdev);
		}
	}

	input_unregister_device(input);

	platform_device_unregister(pdev);

	kfree(old_state);
	return 0;
}

static struct platform_driver encoder_device_driver = {
	.probe		= encoder_probe,
	.remove		= __devexit_p(encoder_remove),
	.driver		= {
		.name	= "encoder gpio",
	}
};

static int __init encoder_init(void)
{
	int ret;

	ret = platform_driver_register(&encoder_device_driver);

	return ret;
}

static void __exit encoder_exit(void)
{
	platform_driver_unregister(&encoder_device_driver);
}

module_init(encoder_init);
module_exit(encoder_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andy <tjin@ingenic.cn>");
MODULE_DESCRIPTION("Rotary encoder driver");
