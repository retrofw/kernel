/* 
 * 
 * drivers/input/touchscreen/gwtc9xxxb_ts.c
 *
 * FocalTech gwtc9xxxb TouchScreen driver. 
 *
 * Copyright (c) 2011  Ingenic Semiconductor Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gwtc9xxxb_ts.h>
//#include <linux/earlysuspend.h>
#include <asm/jzsoc.h>

static struct i2c_client *this_client;
static struct gwtc9xxxb_ts_platform_data *pdata;
extern void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk);
#define CONFIG_GWTC9XXXB_MULTITOUCH 0 //for android is '1' , for linux is '0' 

#define PENUP_TIMEOUT_TIMER 1
#define P2_PACKET_LEN 11
#define P1_PACKET_LEN 5
#define DATABASE 0
struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16 	gesture_code;
	u16	sleep_mode;
    	u16 pressure;
    u8  touch_point;
};

struct gwtc9xxxb_ts_data {
	struct input_dev	*input_dev;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	//struct early_suspend	early_suspend;
	struct mutex lock;
#ifdef PENUP_TIMEOUT_TIMER
		struct timer_list penup_timeout_timer;
#endif
#ifdef CONFIG_GWTC9XXXB_DEBUG
	long int count;
#endif

};

static int gwtc9xxxb_i2c_rxdata(char *rxdata, int length)
{
	int ret;
	struct gwtc9xxxb_ts_data *gwtc9xxxb_ts = i2c_get_clientdata(this_client);
	struct i2c_msg msg1[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
	};
	mutex_lock(&gwtc9xxxb_ts->lock);
	ret = i2c_transfer(this_client->adapter, msg1, 1);
	if (ret < 0){
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
		return ret;
	}
	
	struct i2c_msg msg2[] = {
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};
	ret = i2c_transfer(this_client->adapter, msg2, 1);
	mutex_unlock(&gwtc9xxxb_ts->lock);
	if (ret < 0){
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
		return ret;
	}
	return ret;
}

static int gwtc9xxxb_i2c_txdata(char *txdata, int length)
{
	int ret;
	struct gwtc9xxxb_ts_data *gwtc9xxxb_ts = i2c_get_clientdata(this_client);

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	msleep(1);
	mutex_lock(&gwtc9xxxb_ts->lock);
	ret = i2c_transfer(this_client->adapter, msg, 1);
	mutex_unlock(&gwtc9xxxb_ts->lock);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int gwtc9xxxb_set_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = gwtc9xxxb_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }
    
    return 0;
}
static void gwtc9xxxb_ts_release(struct gwtc9xxxb_ts_data *gwtc9xxxb_ts)
{
#if CONFIG_GWTC9XXXB_MULTITOUCH	
	input_report_abs(gwtc9xxxb_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#else
	input_report_abs(gwtc9xxxb_ts->input_dev, ABS_PRESSURE, 0);
	input_report_key(gwtc9xxxb_ts->input_dev, BTN_TOUCH, 0);
#endif
	input_sync(gwtc9xxxb_ts->input_dev);
#ifdef PENUP_TIMEOUT_TIMER
		del_timer(&(gwtc9xxxb_ts->penup_timeout_timer));
#endif
}

static void gwtc9xxxb_chip_reset(void)
{
	__gpio_as_output(GPIO_GWTC9XXXB_RST);
	__gpio_set_pin(GPIO_GWTC9XXXB_RST);
	msleep(500);
	__gpio_clear_pin(GPIO_GWTC9XXXB_RST);
	 msleep(500);
	__gpio_set_pin(GPIO_GWTC9XXXB_RST);
}

static int gwtc9xxxb_read_data(void)
{
	struct gwtc9xxxb_ts_data *gwtc9xxxb_ts = i2c_get_clientdata(this_client);
	struct ts_event *event = &gwtc9xxxb_ts->event;
	u8 buf[P2_PACKET_LEN + 1] = {0};
	int ret = -1;

	buf[0] = DATABASE;
#if CONFIG_GWTC9XXXB_MULTITOUCH
	ret = gwtc9xxxb_i2c_rxdata(buf, P2_PACKET_LEN);
#else
     	ret = gwtc9xxxb_i2c_rxdata(buf, P1_PACKET_LEN);
#endif
    if (ret < 0) {
		gwtc9xxxb_chip_reset();
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	if((buf[1]==0xff) && (buf[2]==0xff) && (buf[3]==0xff) && (buf[4]==0xff)) {
		//gwtc9xxxb_ts_release(gwtc9xxxb_ts);
		return 1;
	}
	
	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[0] & 0x0f;
	/* for android */
//	event->gesture_code = buf[9];
//	event->sleep_mode = buf[10];

	if ((event->touch_point == 0)) {
		gwtc9xxxb_ts_release(gwtc9xxxb_ts);
		return 1; 
	}

#if CONFIG_GWTC9XXXB_MULTITOUCH
    switch (event->touch_point) {
		case 2:
			event->x2 = ((((u16)buf[5])<<8)&0x0f00) |buf[6];
			event->y2 = ((((u16)buf[7])<<8)&0x0f00) |buf[8];
		case 1:
			event->x1 = ((((u16)buf[1])<<8)&0x0f00) |buf[2];
			event->y1 = ((((u16)buf[3])<<8)&0x0f00) |buf[4];
            break;
		default:
		    return -1;
	}
#else
    if (event->touch_point == 1) {
	event->x1 = ((((u16)buf[1])<<8)&0x0f00) |(u16)buf[2];
	event->y1 = ((((u16)buf[3])<<8)&0x0f00) |(u16)buf[4];
    }
#endif

    	
    	event->pressure = 200;
#ifdef PENUP_TIMEOUT_TIMER
	mod_timer(&(gwtc9xxxb_ts->penup_timeout_timer), jiffies+40);
#endif
//	printk("%d , (%d, %d), (%d, %d)\n", event->touch_point, event->x1, event->y1, event->x2, event->y2);
    return 0;
}

static void gwtc9xxxb_report_value(void)
{
	struct gwtc9xxxb_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
//	printk("--%d , (%d, %d), (%d, %d)\n", event->touch_point, event->x1, event->y1, event->x2, event->y2);
#if CONFIG_GWTC9XXXB_MULTITOUCH
	switch(event->touch_point) {
		case 2:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
		case 1:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
		default:
			break;
	}
#else	/* CONFIG_GWTC9XXXB_MULTITOUCH*/
	if (event->touch_point == 1) {
		input_report_abs(data->input_dev, ABS_X, event->x1);
		input_report_abs(data->input_dev, ABS_Y, event->y1);
		input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
	}
	input_report_key(data->input_dev, BTN_TOUCH, 1);
#endif	/* CONFIG_GWTC9XXXB_MULTITOUCH*/
	input_sync(data->input_dev);

	dev_dbg(&this_client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,
		event->x1, event->y1, event->x2, event->y2);
}	/*end gwtc9xxxb_report_value*/

static void gwtc9xxxb_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	ret = gwtc9xxxb_read_data();
		
	if (ret == 0) {	
		gwtc9xxxb_report_value();
	}
    	enable_irq(this_client->irq);
}
static irqreturn_t gwtc9xxxb_ts_interrupt(int irq, void *dev_id)
{
	struct gwtc9xxxb_ts_data *gwtc9xxxb_ts = dev_id;
	disable_irq_nosync(this_client->irq);

	if (!work_pending(&gwtc9xxxb_ts->pen_event_work)) {
		queue_work(gwtc9xxxb_ts->ts_workqueue, &gwtc9xxxb_ts->pen_event_work);
	}
	
	return IRQ_HANDLED;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void gwtc9xxxb_ts_suspend(struct early_suspend *handler)
{
	struct gwtc9xxxb_ts_data *ts;
	ts =  container_of(handler, struct gwtc9xxxb_ts_data, early_suspend);

#ifdef CONFIG_GWTC9XXXB_DEBUG
	printk("==gwtc9xxxb_ts_suspend=\n");
#endif
	disable_irq(this_client->irq);
	if(cancel_work_sync(&ts->pen_event_work))
		enable_irq(this_client->irq);
	flush_workqueue(ts->ts_workqueue);
    	gwtc9xxxb_set_reg(GWTC9XXXB_REG_SLEEP, SLEEP_MODE);
}

static void gwtc9xxxb_ts_resume(struct early_suspend *handler)
{
#ifdef CONFIG_GWTC9XXXB_DEBUG
	printk("==gwtc9xxxb_ts_resume=\n");
#endif
	__gpio_as_output(pdata->intr);
	__gpio_clear_pin(pdata->intr);
	msleep(10);
	__gpio_as_irq_fall_edge(pdata->intr);

	enable_irq(this_client->irq);
}
#endif  //CONFIG_HAS_EARLYSUSPEND

static void gwtc9xxxb_ts_reset(void)
{
	__gpio_as_output(GPIO_GWTC9XXXB_RST);
	__gpio_set_pin(GPIO_GWTC9XXXB_RST);
	msleep(10);
	__gpio_clear_pin(GPIO_GWTC9XXXB_RST);
	msleep(10);
	__gpio_set_pin(GPIO_GWTC9XXXB_RST);
	msleep(10);
}
static int 
gwtc9xxxb_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct gwtc9xxxb_ts_data *gwtc9xxxb_ts;
	struct input_dev *input_dev;
	int err = 0;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	gwtc9xxxb_ts = kzalloc(sizeof(*gwtc9xxxb_ts), GFP_KERNEL);
	if (!gwtc9xxxb_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	mutex_init(&gwtc9xxxb_ts->lock);
	this_client = client;
	i2c_set_clientdata(client, gwtc9xxxb_ts);
	i2c_jz_setclk(client, 100*1000);

	INIT_WORK(&gwtc9xxxb_ts->pen_event_work, gwtc9xxxb_ts_pen_irq_work);
	gwtc9xxxb_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!gwtc9xxxb_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "%s: platform data is null\n", __func__);
		goto exit_platform_data_null;
	}
	err = request_irq(client->irq, gwtc9xxxb_ts_interrupt, IRQF_DISABLED, "gwtc9xxxb_ts", gwtc9xxxb_ts);
	if (err < 0) {
		dev_err(&client->dev, "gwtc9xxxb_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	
	__gpio_disable_pull(GPIO_GWTC9XXXB_INT);
	__gpio_as_irq_fall_edge(pdata->intr);
	disable_irq(this_client->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	gwtc9xxxb_ts->input_dev = input_dev;
	

#if CONFIG_GWTC9XXXB_MULTITOUCH

	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);

	input_dev->name		= GWTC9XXXB_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"gwtc9xxxb_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	
#ifdef PENUP_TIMEOUT_TIMER
		init_timer(&(gwtc9xxxb_ts->penup_timeout_timer));
		gwtc9xxxb_ts->penup_timeout_timer.data = (unsigned long)gwtc9xxxb_ts;
		gwtc9xxxb_ts->penup_timeout_timer.function  =	(void (*)(unsigned long))gwtc9xxxb_ts_release;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	gwtc9xxxb_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	gwtc9xxxb_ts->early_suspend.suspend = gwtc9xxxb_ts_suspend;
	gwtc9xxxb_ts->early_suspend.resume	= gwtc9xxxb_ts_resume;
	register_early_suspend(&gwtc9xxxb_ts->early_suspend);
#endif
		
	gwtc9xxxb_ts_reset();

    	enable_irq(this_client->irq);

    return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, gwtc9xxxb_ts);
exit_irq_request_failed:
exit_platform_data_null:
	cancel_work_sync(&gwtc9xxxb_ts->pen_event_work);
	destroy_workqueue(gwtc9xxxb_ts->ts_workqueue);
exit_create_singlethread:
	i2c_set_clientdata(client, NULL);
	kfree(gwtc9xxxb_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit gwtc9xxxb_ts_remove(struct i2c_client *client)
{
	struct gwtc9xxxb_ts_data *gwtc9xxxb_ts = i2c_get_clientdata(client);
//	unregister_early_suspend(&gwtc9xxxb_ts->early_suspend);
	free_irq(client->irq, gwtc9xxxb_ts);
	input_unregister_device(gwtc9xxxb_ts->input_dev);
	kfree(gwtc9xxxb_ts);
	cancel_work_sync(&gwtc9xxxb_ts->pen_event_work);
	destroy_workqueue(gwtc9xxxb_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id gwtc9xxxb_ts_id[] = {
	{ GWTC9XXXB_NAME, 0 },
};
MODULE_DEVICE_TABLE(i2c, gwtc9xxxb_ts_id);

static struct i2c_driver gwtc9xxxb_ts_driver = {
	.probe		= gwtc9xxxb_ts_probe,
	.remove		= __devexit_p(gwtc9xxxb_ts_remove),
	.id_table	= gwtc9xxxb_ts_id,
	.driver	= {
		.name	= GWTC9XXXB_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init gwtc9xxxb_ts_init(void)
{
	return i2c_add_driver(&gwtc9xxxb_ts_driver);
}

static void __exit gwtc9xxxb_ts_exit(void)
{
	i2c_del_driver(&gwtc9xxxb_ts_driver);
}

module_init(gwtc9xxxb_ts_init);
module_exit(gwtc9xxxb_ts_exit);

MODULE_AUTHOR("<bcjia@ingenic.cn>");
MODULE_DESCRIPTION("FocalTech gwtc9xxxb TouchScreen driver");
MODULE_LICENSE("GPL");
