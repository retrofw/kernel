/**
 * Copyright 2013, MEMS Vision Worldwide.
 * Copyright 2013, Ingenic Semiconductor.
 *
 * Project: MVH3000D Linux Driver (V1.0)
 * File: mvh3000d_i2c.c
 * Author: MV Support
 *         Aaron Wang <hfwang@ingenic.cn>
 *
 * Platform: Linux or android
 * Development Environment: Sitara Linux SDK 3.2.0 or andoird4.3
 *
 * Description: This file provides the driver to communicate with
 *
 * The sensor on the i2c bus. It generates two userspace
 * files from which the humidity and temperature values
 * can be retrieved.  This code was written to demonstrate
 * how to communicate with the sensor, and not for efficiency.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/input.h>

#define DRV_NAME 	"mvh3000d_i2c"
#define DUMMY_DATA  	0x00
#define DATA_LENGTH 	4
#define MEASURE_DELAY	50

extern void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk);
/* Minimal time (0.5s) to wait between pollings [to prevent self-heating of the sensor] */
#define MIN_POLL_TIME	(int)(0.5*HZ)
enum {
	SENSOR_TEMP = 0,
	SENSOR_HUMI = 1,
};

struct mvh3000d {
	struct i2c_client *client;
	struct device *hwmon_dev;  // Hardware device structure
	struct mutex lock;         // Mutex
	char hasStarted;           // Status byte
	unsigned long last_probe;  // Storage of the last probing time
	int temperature;           // Current temperature value
	int humidity;              // Current relative humidity value

	struct delayed_work input_twork;
	struct delayed_work input_hwork;

	struct input_dev *input_tdev;
	struct input_dev *input_hdev;

	unsigned int poll_interval_temp;
	unsigned int poll_interval_humi;

	atomic_t enabled;
	atomic_t temp_enabled;
	atomic_t humi_enabled;
};

static int mvh3000d_enable(struct mvh3000d *mvh3000d, int sensor);
static int mvh3000d_disable(struct mvh3000d *mvh3000d, int sensor);
/*
 * Function to command the sensor to initiate a measurement
 * Inputs: I2C client
 * Outputs: Communication status
 *
 * As stated in section 5.2 of the MVH3000D datasheet, to initiate a measurement,
 * we send the I2C address with a write bit. Since we don't need to write an
 * additional value, we send a dummy byte (0x00)
 */

static inline s32 initiateMeasurement(struct i2c_client *client)
{
	int ret;

	/* Write the dummy byte to the i2c bus */
	ret = i2c_smbus_write_byte(client, DUMMY_DATA);
	/*
	 * Return communication status
	 * -- Zero on success
	 * -- Negative error code on failure
	 */
	return ret;
}

/*
 * Function to command the sensor to read a measurement
 * Inputs: I2C client
 * Outputs: Communication status
 *
 * As stated in section 5.2 of the MVH3000D datasheet, to read a measurement, we send the I2C
 * address with a read bit, then read the four following bytes
 */

static inline s32 readMeasurement(struct i2c_client *client, char *data)
{
	int ret;

	/* Read 4 bytes over the i2c bus and store them into the data buffer */
	ret = i2c_master_recv(client, data, DATA_LENGTH);

	/*
	 * Return communication status
	 * -- Number of bytes read on success
	 * -- Negative error code on failure
	 */
	return ret;
}

/*
 * Function to convert the raw temperature data into degrees Celsius multiplied by 100 to avoid floating points
 * Inputs: MSB and LSB of raw temperature data
 * Outputs: Integer in degrees Celsius * 100
 *
 * As stated in section 5.2 of the MVH3000D datasheet, to convert temperature from raw data to
 * degrees Celsius we need to apply the following equation:
 *
   /  Traw    \
   T[degC] =  | ---------  |  X 165  - 40
   \ 2^14 - 1 /
 *
 * However, since the Linux kernel should not perform floating point
 * operations we are multiplying it by 100. Therefore the output for
 * 42.75°C would be "4275".
*/

static inline int mvh3000d_convert_temperature(char msb, char lsb)
{
	/* Format data */
	int value = ((msb << 8) | (lsb & 0x3F));

	/* Rescale data */
	value = (16500 * value) / ( 4* ((1 << 14) - 1)) - 4000;

	/* Return converted value */
	return value;
}

/*
 * Function to convert the raw humidity data into percent RH multiplied by 100 to avoid floating points
 * Inputs: MSB and LSB of raw humidity data
 * Outputs: Integer in percent RH * 100

 * As stated in section 5.2 of the MVH3000D datasheet, to convert relative humidity from raw data to
 * percent RH we need to apply the following equation
 *
   /  Hraw    \
   H[%RH] =   | ---------  |  X 100
   \ 2^14 - 1 /
 *
 * However, since the Linux kernel should not perform floating point
 * operations we are multiplying it by 100. Therefore the output for
 * 65.23%RH would be "6523".
 */

static inline int mvh3000d_convert_humidity(char msb, char lsb)
{
	/* Format data */
	int value = (((msb & 0x3F) << 8) | (lsb & 0xff)) & 0x3fff;

	/* Rescale data */
	value = (10000 * value) / ( (1 << 14) - 1);
	/* Return converted value */
	return value;
}

/*
 * Function managing the data retrieval from the sensor
 * Inputs: I2C client
 * Outputs: None
 *
 * The function manages the initiation of a measurement to the sensors, waits 50 ms
 * for the measurement to be completed (assumes 14-bit measurement resolution), then read the updated
 * sensor data. Finally, it convert data to either degrees Celsius or percent RH (times 100)
 * before storing it in the device's appropriate structure members
 */

static void mvh3000d_get_data(struct i2c_client *client)
{

	int ret = 0;
	char dataTransfert[4];
	struct mvh3000d *mvh3000d =  i2c_get_clientdata(client);

	mutex_lock(&mvh3000d->lock);

	/*
	 * If the sensor has not yet provided a value, or if it has been long enough since last polling.
	 * This procedure prevents self-heating from the sensor, which would influence its output.
	 */
	if ( !mvh3000d->hasStarted || time_after(jiffies, mvh3000d->last_probe + MIN_POLL_TIME ) ) {

		/* Tell the sensor we want data */
		ret = initiateMeasurement(client);

		/* Wait for measurement */
		msleep(MEASURE_DELAY);

		/* Retrieve measurement */
		ret = readMeasurement(client, dataTransfert);

		/* Process and store data */
		mvh3000d->humidity = mvh3000d_convert_humidity(dataTransfert[0], dataTransfert[1]);
		mvh3000d->temperature = mvh3000d_convert_temperature(dataTransfert[2], dataTransfert[3]);

		/* Set the time */
		mvh3000d->last_probe = jiffies;

		/* Set the flag */
		mvh3000d->hasStarted = 1;
	}

	mutex_unlock(&mvh3000d->lock);

	return;
}

/*
 * Function that gets temperature data from the device's structure and write it to a file
 * Inputs: hwmon device, associated attribute, buffer to write to
 * Outputs: Status of the printing
 *
 * Note that for a 42.75°C value the actual output will be "4275".
 */

static ssize_t mvh3000d_show_temperature(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct mvh3000d *mvh3000d = i2c_get_clientdata(client);

	mvh3000d_get_data(client);

	ret = sprintf(buf, "%d\n", mvh3000d->temperature);

	return ret;
}

/*
 * Function that gets humidity data from the device's structure and write it to a file
 * Inputs: hwmon device, associated attribute, buffer to write to
 * Outputs: Status of the printing
 *
 * Note that for a 65.23%RH value the actual output will be"6523".
 */

static ssize_t mvh3000d_show_humidity(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct mvh3000d *mvh3000d = i2c_get_clientdata(client);

	mvh3000d_get_data(client);

	ret = sprintf(buf, "%d\n", mvh3000d->humidity);

	return ret;
}

static ssize_t mvh3000d_show_enable_temp(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct mvh3000d *mvh3000d = i2c_get_clientdata(client);

	ret = sprintf(buf, "%d\n", atomic_read(&mvh3000d->temp_enabled));

	return ret;
}

static ssize_t mvh3000d_store_enable_temp(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret;
	unsigned long enable;
	struct i2c_client *client = to_i2c_client(dev);
	struct mvh3000d *mvh3000d = i2c_get_clientdata(client);

	if (strict_strtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable) {
		ret = mvh3000d_enable(mvh3000d, SENSOR_TEMP);
	} else {
		ret = mvh3000d_disable(mvh3000d, SENSOR_TEMP);
	}

	return size;
}

static ssize_t mvh3000d_show_enable_humi(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct mvh3000d *mvh3000d = i2c_get_clientdata(client);

	ret = sprintf(buf, "%d\n", atomic_read(&mvh3000d->humi_enabled));

	return ret;
}

static ssize_t mvh3000d_store_enable_humi(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret;
	unsigned long enable;
	struct i2c_client *client = to_i2c_client(dev);
	struct mvh3000d *mvh3000d = i2c_get_clientdata(client);

	if (strict_strtoul(buf, 10, &enable))
		return -EINVAL;

	if (enable) {
		ret = mvh3000d_enable(mvh3000d, SENSOR_HUMI);
	} else {
		ret = mvh3000d_disable(mvh3000d, SENSOR_HUMI);
	}

	return size;
}

static ssize_t mvh3000d_show_temp_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct i2c_client *client = to_i2c_client(dev);
	struct mvh3000d *mvh3000d = i2c_get_clientdata(client);

	mutex_lock(&mvh3000d->lock);
	val = mvh3000d->poll_interval_temp;
	mutex_unlock(&mvh3000d->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t mvh3000d_store_temp_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned long interval_ms;
	struct i2c_client *client = to_i2c_client(dev);
	struct mvh3000d *mvh3000d = i2c_get_clientdata(client);

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;

	mutex_lock(&mvh3000d->lock);
	mvh3000d->poll_interval_temp = (unsigned int)interval_ms;
	mutex_unlock(&mvh3000d->lock);
	return size;
}

static ssize_t mvh3000d_show_humi_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct i2c_client *client = to_i2c_client(dev);
	struct mvh3000d *mvh3000d = i2c_get_clientdata(client);

	mutex_lock(&mvh3000d->lock);
	val = mvh3000d->poll_interval_humi;
	mutex_unlock(&mvh3000d->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t mvh3000d_store_humi_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	unsigned long interval_ms;
	struct i2c_client *client = to_i2c_client(dev);
	struct mvh3000d *mvh3000d = i2c_get_clientdata(client);

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;

	mutex_lock(&mvh3000d->lock);
	mvh3000d->poll_interval_humi = (unsigned int)interval_ms;
	mutex_unlock(&mvh3000d->lock);
	return size;
}

static SENSOR_DEVICE_ATTR(temperature, S_IRUGO, mvh3000d_show_temperature,
		NULL, 0);

static SENSOR_DEVICE_ATTR(humidity, S_IRUGO, mvh3000d_show_humidity,
		NULL, 0);

static SENSOR_DEVICE_ATTR(enable_temperature, S_IRUGO | S_IWUGO | S_IXUGO,
		mvh3000d_show_enable_temp, mvh3000d_store_enable_temp, 0);

static SENSOR_DEVICE_ATTR(enable_humidity, S_IRUGO | S_IWUGO | S_IXUGO,
		mvh3000d_show_enable_humi, mvh3000d_store_enable_humi, 0);

static SENSOR_DEVICE_ATTR(poll_period_ms_temp, S_IRUGO | S_IWUGO | S_IXUGO,
		mvh3000d_show_temp_polling_rate, mvh3000d_store_temp_polling_rate, 0);

static SENSOR_DEVICE_ATTR(poll_period_ms_humi, S_IRUGO | S_IWUGO | S_IXUGO,
		mvh3000d_show_humi_polling_rate, mvh3000d_store_humi_polling_rate, 0);

static struct attribute *mvh3000d_attributes[] = {
	&sensor_dev_attr_temperature.dev_attr.attr,
	&sensor_dev_attr_humidity.dev_attr.attr,
	&sensor_dev_attr_enable_temperature.dev_attr.attr,
	&sensor_dev_attr_enable_humidity.dev_attr.attr,
	&sensor_dev_attr_poll_period_ms_temp.dev_attr.attr,
	&sensor_dev_attr_poll_period_ms_humi.dev_attr.attr,
	NULL
};

static const struct attribute_group mvh3000d_attribute_group = {
	.attrs = mvh3000d_attributes,
};

static void mvh3000d_device_power_off(struct mvh3000d *mvh3000d)
{
	atomic_set(&mvh3000d->enabled, 0);
}

static int mvh3000d_device_power_on(struct mvh3000d *mvh3000d)
{
	atomic_set(&mvh3000d->enabled, 1);

	return 0;
}

static int mvh3000d_enable(struct mvh3000d *mvh3000d, int sensor)
{
	int err;

	switch (sensor) {
		case SENSOR_TEMP:
			if (!atomic_cmpxchg(&mvh3000d->temp_enabled, 0, 1)) {
				err = mvh3000d_device_power_on(mvh3000d);
				if (err < 0) {
					atomic_set(&mvh3000d->temp_enabled, 0);
					return err;
				}
				schedule_delayed_work(&mvh3000d->input_twork,
						msecs_to_jiffies(mvh3000d->poll_interval_temp));
			}
			break;

		case SENSOR_HUMI:
			if (!atomic_cmpxchg(&mvh3000d->humi_enabled, 0, 1)) {
				err = mvh3000d_device_power_on(mvh3000d);
				if (err < 0) {
					atomic_set(&mvh3000d->humi_enabled, 0);
					return err;
				}
				schedule_delayed_work(&mvh3000d->input_hwork,
						msecs_to_jiffies(mvh3000d->poll_interval_humi));
			}
			break;
		default:
			break;
	}

	return 0;
}

static int mvh3000d_disable(struct mvh3000d *mvh3000d, int sensor)
{
	switch (sensor) {
		case SENSOR_TEMP:
			if (atomic_cmpxchg(&mvh3000d->temp_enabled, 1, 0)) {
				cancel_delayed_work_sync(&mvh3000d->input_twork);
			}
			break;

		case SENSOR_HUMI:
			if (atomic_cmpxchg(&mvh3000d->humi_enabled, 1, 0)) {
				cancel_delayed_work_sync(&mvh3000d->input_hwork);
			}
			break;
		default:
			break;
	}

	if (!atomic_read(&mvh3000d->humi_enabled) && !atomic_read(&mvh3000d->temp_enabled)) {
		mvh3000d_device_power_off(mvh3000d);
	}

	return 0;
}

static void mvh3000d_input_hwork_func(struct work_struct *work)
{
	struct mvh3000d *mvh3000d = container_of((struct delayed_work *)work,
			struct mvh3000d, input_hwork);

	mvh3000d_get_data(mvh3000d->client);
	input_report_abs(mvh3000d->input_hdev, ABS_MISC, mvh3000d->humidity);
	input_sync(mvh3000d->input_hdev);

	schedule_delayed_work(&mvh3000d->input_hwork,
			msecs_to_jiffies(mvh3000d->poll_interval_humi));
}

static void mvh3000d_input_twork_func(struct work_struct *work)
{
	struct mvh3000d *mvh3000d = container_of((struct delayed_work *)work,
			struct mvh3000d, input_twork);

	mvh3000d_get_data(mvh3000d->client);
	input_report_abs(mvh3000d->input_tdev, ABS_GAS, mvh3000d->temperature);
	input_sync(mvh3000d->input_tdev);

	schedule_delayed_work(&mvh3000d->input_twork,
			msecs_to_jiffies(mvh3000d->poll_interval_temp));
}

static int mvh3000d_input_hdev_init(struct mvh3000d *mvh3000d)
{
	int err;

	INIT_DELAYED_WORK(&mvh3000d->input_hwork, mvh3000d_input_hwork_func);

	mvh3000d->input_hdev = input_allocate_device();
	if (!mvh3000d->input_hdev) {
		err = -ENOMEM;
		dev_err(&mvh3000d->client->dev, "input device allocate failed\n");
		goto err0;
	}

	mvh3000d->input_hdev->name = "mvh3000d_humi";
	mvh3000d->input_hdev->id.bustype = BUS_I2C;
	mvh3000d->input_hdev->dev.parent = &mvh3000d->client->dev;

	input_set_drvdata(mvh3000d->input_hdev, mvh3000d);

	set_bit(EV_ABS, mvh3000d->input_hdev->evbit);

	input_set_abs_params(mvh3000d->input_hdev, ABS_MISC, 0, 100, 0, 0);

	err = input_register_device(mvh3000d->input_hdev);
	if (err) {
		dev_err(&mvh3000d->client->dev,
			"unable to register input polled device %s\n",
			mvh3000d->input_hdev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(mvh3000d->input_hdev);
err0:
	return err;
}

static int mvh3000d_input_tdev_init(struct mvh3000d *mvh3000d)
{
	int err;

	INIT_DELAYED_WORK(&mvh3000d->input_twork, mvh3000d_input_twork_func);

	mvh3000d->input_tdev = input_allocate_device();
	if (!mvh3000d->input_tdev) {
		err = -ENOMEM;
		dev_err(&mvh3000d->client->dev, "input device allocate failed\n");
		goto err0;
	}

	mvh3000d->input_tdev->name = "mvh3000d_temp";
	mvh3000d->input_tdev->id.bustype = BUS_I2C;
	mvh3000d->input_tdev->dev.parent = &mvh3000d->client->dev;

	input_set_drvdata(mvh3000d->input_tdev, mvh3000d);

	set_bit(EV_ABS, mvh3000d->input_tdev->evbit);

	input_set_abs_params(mvh3000d->input_tdev, ABS_GAS,
			-40, 125, 0, 0);

	err = input_register_device(mvh3000d->input_tdev);
	if (err) {
		dev_err(&mvh3000d->client->dev,
			"unable to register input polled device %s\n",
			mvh3000d->input_tdev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(mvh3000d->input_tdev);
err0:
	return err;
}

static void mvh3000d_input_cleanup(struct mvh3000d *mvh3000d)
{
	input_unregister_device(mvh3000d->input_tdev);
	input_unregister_device(mvh3000d->input_hdev);
}

static int __devinit mvh3000d_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mvh3000d *mvh3000d;
	int err;
	uint8_t buf[2] = { 0 };

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto fail_check_i2c;
	}

	mvh3000d = kzalloc(sizeof(*mvh3000d), GFP_KERNEL);
	if (!mvh3000d) {
		dev_dbg(&client->dev, "kzalloc failed\n");
		err = -ENOMEM;
		goto fail_check_i2c;
	}

	mutex_init(&mvh3000d->lock);
	mutex_lock(&mvh3000d->lock);

	mvh3000d->client = client;
	mvh3000d->poll_interval_temp = CONFIG_MVH3004D_POLL_INTERVAL;
	mvh3000d->poll_interval_humi = CONFIG_MVH3004D_POLL_INTERVAL;

	i2c_set_clientdata(client, mvh3000d);
        i2c_jz_setclk(client, 10*1000);

	mvh3000d_device_power_on(mvh3000d);

	/* detect sensor */
	buf[0] = DUMMY_DATA;
	err = i2c_master_send(client, buf, 1);
	if (err < 0) {
		dev_err(&client->dev, "send one byte failed !\n");
		goto fail_detect_sensor;
	}

	err = mvh3000d_input_tdev_init(mvh3000d);
	if (err < 0) {
		dev_err(&client->dev, "input temp device init failed\n");
		goto fail_input_tdev;
	}

	err = mvh3000d_input_hdev_init(mvh3000d);
	if (err < 0) {
		dev_err(&client->dev, "input humi device init failed\n");
		goto fail_input_hdev;
	}

	err = sysfs_create_group(&client->dev.kobj, &mvh3000d_attribute_group);
	if (err) {
		dev_dbg(&client->dev, "could not create sysfs files\n");
		goto fail_free;
	}

	mvh3000d->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(mvh3000d->hwmon_dev)) {
		dev_dbg(&client->dev, "unable to register hwmon device\n");
		err = PTR_ERR(mvh3000d->hwmon_dev);
		goto fail_remove_sysfs;
	}

	dev_info(&client->dev, "initialized\n");
	mvh3000d_device_power_off(mvh3000d);

	mutex_unlock(&mvh3000d->lock);

	return 0;

fail_remove_sysfs:
	sysfs_remove_group(&client->dev.kobj, &mvh3000d_attribute_group);
fail_free:
	input_unregister_device(mvh3000d->input_hdev);
fail_input_hdev:
	input_unregister_device(mvh3000d->input_tdev);
fail_detect_sensor:
fail_input_tdev:
	mvh3000d_device_power_off(mvh3000d);
	i2c_set_clientdata(client, NULL);
	mutex_unlock(&mvh3000d->lock);
	kfree(mvh3000d);
fail_check_i2c:
	return err;
}

static int __devexit mvh3000d_remove(struct i2c_client *client)
{
	struct mvh3000d *mvh3000d = i2c_get_clientdata(client);
	hwmon_device_unregister(mvh3000d->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &mvh3000d_attribute_group);
        mvh3000d_input_cleanup(mvh3000d);
	kfree(mvh3000d);

	return 0;
}

static const struct i2c_device_id mvh3000d_id[] = {
	{ "mvh3000d", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mvh3000d_id);

static struct i2c_driver mvh3000d_i2c_driver = {
	.driver.name = "mvh3000d",
	.probe       = mvh3000d_probe,
	.remove      = __devexit_p(mvh3000d_remove),
	.id_table    = mvh3000d_id,
};

static int __init mvh3000d_init(void)
{
	return i2c_add_driver(&mvh3000d_i2c_driver);    
}

static void __exit mvh3000d_exit(void)
{
	i2c_del_driver(&mvh3000d_i2c_driver);        
}

MODULE_AUTHOR("MV Support <support@mems-vision.com>"
		"Aaron Wang <hfwang@ingenic.cn>");
MODULE_DESCRIPTION("MEMS-Vision MVH3000D Humidity and Temperature Sensor driver");
MODULE_LICENSE("GPL");

module_init(mvh3000d_init);
module_exit(mvh3000d_exit);
