/*
 * linux/drivers/input/keyboard/jz_gpio_keys.c
 *
 * Keypad driver based on GPIO pins for Jz4750 APUS board.
 *
 * User applications can access to this device via /dev/input/eventX.
 *
 * Copyright (c) 2005 - 2009  Ingenic Semiconductor Inc.
 *
 * Author: Richard <cjfeng@ingenic.cn>
 *         Regen   <lhhuang@ingenic.cn>
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
//#include <linux/wakelock.h>

#include <linux/semaphore.h>

#include <asm/gpio.h>
#include <asm/jzsoc.h>


#undef DEBUG
//#define DEBUG 

#ifdef DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...)
#endif

/* Timer interval */
#define SCAN_INTERVAL	5
#define MAX_KEY_NUM     12

//#define KERNEL_HIBERNATE_TIME (10*HZ)
#define KERNEL_HIBERNATE_TIME (5*HZ)

#if defined(GPIO_ENDCALL_TCU_CHANNEL)
#define ENDCALL_CHANNEL GPIO_ENDCALL_TCU_CHANNEL
#else
#define ENDCALL_CHANNEL 3
#endif

//static struct timer_list kbd_timer[KEY_NUM];
//static int current_key[KEY_NUM];

static struct timer_list *kbd_timer;
static struct timer_list endcall_timer;
static int *current_key;

static struct input_dev *ginput;
static int endcall_index;
struct semaphore sem;
struct workqueue_struct *endcall_wqueue;
struct work_struct endcall_irq_work;
//static struct wake_lock delay_wake_lock;

static void enable_gpio_irqs(struct gpio_keys_platform_data *pdata)
{
	int i;

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];

		if (button->active_low)
			__gpio_as_irq_fall_edge(button->gpio);
		else
			__gpio_as_irq_rise_edge(button->gpio);
	}
}
#if 0
static void disable_gpio_irqs(struct gpio_keys_platform_data *pdata)
{
	int i;

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		__gpio_mask_irq(button->gpio);
	}
}
static void gpio_keys_shutdown(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	//Disable all gpio key irq
	printk("Disable all gpio key irq\n");
	disable_gpio_irqs(pdata);
}
#endif

static void button_timer_callback(unsigned long data)
{
	int gpio;
	int code;
	int active_low;
	struct platform_device *pdev = (struct platform_device *)data;
	struct input_dev *input = platform_get_drvdata(pdev);
	struct gpio_keys_platform_data *key_data = 
		(struct gpio_keys_platform_data*)pdev->dev.platform_data;
	struct gpio_keys_button *board_buttons = key_data->buttons;

	static int button_pressed[MAX_KEY_NUM] = { 0, 0, 0, 0, 0 };
	int state, i;
	down(&sem);

	for (i = 0; i < key_data->nbuttons; i++) {
		if (1 == current_key[i]) {
			gpio = board_buttons[i].gpio;
			code = board_buttons[i].code;
			active_low = board_buttons[i].active_low;

			state = __gpio_get_pin(gpio);

			if (active_low ^ state) {
				if(button_pressed[i] != 1) {
					/* button pressed */
					button_pressed[i] = 1;
					input_report_key(input, code, 1);
//					wake_lock_timeout(&delay_wake_lock,5*HZ);
					input_sync(input);
					printk("gpio %d down, code:%d \n",
								gpio, code);
				}
				mod_timer(&kbd_timer[i],
					  jiffies + SCAN_INTERVAL);
			} else {
				/* button released */
				if (1 == button_pressed[i]) {
					input_report_key(input, code, 0);
//					wake_lock_timeout(&delay_wake_lock,5*HZ);
					input_sync(input);
					button_pressed[i] = 0;
					current_key[i] = 0;
					printk("gpio %d up, code:%d \n",
								gpio, code);
				}
			}
		}
	}
	up(&sem);
}

static inline void endcall_report(struct gpio_keys_button *button)
{
	int state;
	static unsigned int last_time = 0xffffffff;
	unsigned int endcall_tmp;

	state = __gpio_get_pin(button->gpio);
	mdelay(5);
	state = __gpio_get_pin(button->gpio);
	
	if(state)
		__gpio_as_irq_fall_edge(button->gpio);
	else
		__gpio_as_irq_rise_edge(button->gpio);



	if(button->active_low ^ state) {
		current_key[endcall_index] = 1;
		mod_timer(&kbd_timer[endcall_index], jiffies + SCAN_INTERVAL);
		mod_timer(&endcall_timer, jiffies + KERNEL_HIBERNATE_TIME); 
		last_time = rtc_read_reg(RTC_RTCSR); 
		dprintk("%x\n",last_time);
	} else {
		if(timer_pending(&endcall_timer))
			del_timer(&endcall_timer);
		if(last_time == 0xffffffff) return;
		endcall_tmp = rtc_read_reg(RTC_RTCSR); 

		dprintk("%x %x\n",last_time,endcall_tmp);
		if(endcall_tmp - last_time > 7)
			queue_work(endcall_wqueue,&endcall_irq_work);
		last_time = 0xffffffff;
	}

}

static irqreturn_t jz_tcu2_interrupt(int irq, void *arg)
{
	struct gpio_keys_button *button = arg;

	if(__tcu_full_match_flag(ENDCALL_CHANNEL))
	{
		disable_irq_nosync(IRQ_TCU2);
	
		__tcu_mask_full_match_irq(ENDCALL_CHANNEL);
		__tcu_stop_counter(ENDCALL_CHANNEL);
		__tcu_disable_pclk(ENDCALL_CHANNEL);
		__tcu_clear_counter_to_zero(ENDCALL_CHANNEL);
		__tcu_clear_full_match_flag(ENDCALL_CHANNEL);
	
		endcall_report(button);
			
		enable_irq(IRQ_TCU2);
		enable_irq(IRQ_GPIO_0 + button->gpio);
	}
	return IRQ_HANDLED;
}

static irqreturn_t jz_endcall_interrupt(int irq, void *arg)
{
	struct gpio_keys_button *button = arg;
	disable_irq_nosync(IRQ_GPIO_0 + button->gpio);

	__tcu_select_clk_div1024(ENDCALL_CHANNEL);
	__tcu_unmask_full_match_irq(ENDCALL_CHANNEL);
	__tcu_select_pclk(ENDCALL_CHANNEL);
	__tcu_clear_counter_to_zero(ENDCALL_CHANNEL);
	__tcu_disable_pclk(ENDCALL_CHANNEL);
	__tcu_set_full_data(ENDCALL_CHANNEL,0x7f);// 1.x ms
	__tcu_select_pclk(ENDCALL_CHANNEL);
	__tcu_start_counter(ENDCALL_CHANNEL);
	return IRQ_HANDLED;
}


static irqreturn_t jz_gpio_interrupt(int irq, void *dev_id)
{
	int i;
	struct platform_device *pdev = dev_id;
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		int gpio = button->gpio;

		if (irq == (gpio + IRQ_GPIO_0) ) {
			current_key[i] = 1;
			/* start timer */
			mod_timer(&kbd_timer[i], jiffies + SCAN_INTERVAL);
			break;
		}
	}

	return IRQ_HANDLED;
}
static int is_hibernate = 0;
static int  kernel_quick_hibernate(void)
{
#if 0
#ifdef CONFIG_JZ4760B_GM_TEST1
	/* power down LED light */
	__gpio_as_output(46);
	__gpio_set_pin(46);

	__gpio_as_output(134);
	__gpio_set_pin(134);

	/* power down IC */
	__gpio_as_output(32 * 1 + 10);
	__gpio_clear_pin(32 * 1 + 10);
	//__gpio_as_input(32 * 1 + 10);   //resolve CPU restart problem when push dow buttom long time.  
	printk("clear PB10 and power off wifi_box\n");
	return 0;
#endif
#endif
	if(is_hibernate){
		printk("Hibernating...\n");
		return -1;
	}
	is_hibernate = 1;
	printk("Kernel ready to hibernate!!!\n");
	pm_power_off();

	/* shouldn't come here */
	return 0;	
}
static void endcall_work_handler(void)
{
	kernel_quick_hibernate();
}
static void encall_timer_callback(unsigned long data)
{
	queue_work(endcall_wqueue,&endcall_irq_work);
}
static int __devinit gpio_keys_probe(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;

	input = input_allocate_device();
	if (!input)
		return -ENOMEM;
	ginput = input;

	if (pdata->nbuttons < 1) {
		printk("%s %d no key register in!\n",__FUNCTION__, __LINE__);
		return -EINVAL;
	}

	current_key = kzalloc(sizeof(int) * pdata->nbuttons, GFP_KERNEL);
	if (!current_key) {
		printk("%s %d no memory alloc for key!\n",__FUNCTION__, __LINE__);
		return -ENOMEM;
	}

	kbd_timer = kzalloc(sizeof(struct timer_list) * pdata->nbuttons, GFP_KERNEL);
	if (!kbd_timer) {
		printk("%s %d no memory alloc for key timer!\n",__FUNCTION__, __LINE__);
		return -ENOMEM;
	}

	sema_init(&sem,1);
	endcall_wqueue = create_workqueue("endcall_queue");
	if (!endcall_wqueue) {
		BUG();	
		}

	INIT_WORK(&endcall_irq_work, (void(*))endcall_work_handler);

	platform_set_drvdata(pdev, input);

	input->name = pdev->name;
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	input->evbit[0] = BIT(EV_KEY) | BIT(EV_SYN);

	set_bit(KEY_MENU, input->keybit);
	set_bit(KEY_HOME, input->keybit);
	set_bit(KEY_SEND, input->keybit);
	set_bit(KEY_BACK, input->keybit);
	set_bit(KEY_END, input->keybit);
	set_bit(KEY_VOLUMEDOWN, input->keybit);
	set_bit(KEY_VOLUMEUP, input->keybit);
	set_bit(KEY_UP, input->keybit);
	set_bit(KEY_DOWN, input->keybit);
	set_bit(KEY_LEFT, input->keybit);
	set_bit(KEY_RIGHT, input->keybit);
	set_bit(KEY_OK, input->keybit);

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		int irq;
		unsigned int type = button->type ?: EV_KEY;

		/* Init timer */
		setup_timer(&kbd_timer[i],
			    button_timer_callback,
			    (unsigned long)pdev);

		irq = IRQ_GPIO_0 + button->gpio;
		if (irq < 0) {
			error = irq;
			pr_err("%s: Unable to get irq number"
			       " for GPIO %d, error %d\n", pdev->name,
				button->gpio, error);
			goto fail;
		}

	//	if (button->code == KEY_END)
		if (button->code == KEY_GOTO)
		{
			setup_timer(&endcall_timer,encall_timer_callback,0);

			error = request_irq(irq, jz_endcall_interrupt,
				    IRQF_SAMPLE_RANDOM | IRQF_DISABLED,
				    button->desc ? button->desc : "gpio_keys",
				    button);
			error+= request_irq(IRQ_TCU2, jz_tcu2_interrupt,
				    IRQF_SAMPLE_RANDOM | IRQF_DISABLED,"gpio_endcall_tcu2",
				    button);
			enable_irq_wake(irq);
			endcall_index = i;
		}
		else
			error = request_irq(irq, jz_gpio_interrupt,
				    IRQF_SAMPLE_RANDOM | IRQF_DISABLED,
				    button->desc ? button->desc : "gpio_keys",
				    pdev);

		if (error) {
			pr_err("%s: Unable to claim irq %d; error %d\n",
			       pdev->name, irq, error);
			goto fail;
		}

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, type, button->code);
	}

	/* Enable all GPIO irqs */
	enable_gpio_irqs(pdata);

	error = input_register_device(input);
	if (error) {
		pr_err("%s: Unable to register input device, "
		       "error: %d\n", pdev->name, error);
		goto fail;
	}

	device_init_wakeup(&pdev->dev, wakeup);
//	wake_lock_init(&delay_wake_lock, WAKE_LOCK_SUSPEND, "jz_gpio_key_delay_wake_lock");

	return 0;

 fail:
	while (--i >= 0) {
		free_irq(pdata->buttons[i].gpio + IRQ_GPIO_0 , pdev);
	}

	platform_set_drvdata(pdev, NULL);
	input_free_device(input);

	return error;
}

static int __devexit gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input = platform_get_drvdata(pdev);
	int i;

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = pdata->buttons[i].gpio + IRQ_GPIO_0;
		free_irq(irq, pdev);
	}

	input_unregister_device(input);

	platform_device_unregister(pdev);

	if (current_key)
		kfree(current_key);
	if (kbd_timer)
		kfree(kbd_timer);

	return 0;
}

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= __devexit_p(gpio_keys_remove),
	.driver		= {
		.name	= "jz-gpio-key",
	}
};

static int __init gpio_keys_init(void)
{
	int ret;

	ret = platform_driver_register(&gpio_keys_device_driver);

	return ret;
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Regen Huang <lhhuang@ingenic.cn>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
