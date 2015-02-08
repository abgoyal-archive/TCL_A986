

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>

/* How many retries on register read error */
#define MAX_RETRIES	5


static const unsigned short normal_i2c[] = { 0x2e, I2C_CLIENT_END };


#define W83L785TS_REG_MAN_ID1		0x4D
#define W83L785TS_REG_MAN_ID2		0x4C
#define W83L785TS_REG_CHIP_ID		0x4E
#define W83L785TS_REG_CONFIG		0x40
#define W83L785TS_REG_TYPE		0x52
#define W83L785TS_REG_TEMP		0x27
#define W83L785TS_REG_TEMP_OVER		0x53 /* not sure about this one */


#define TEMP_FROM_REG(val)	((val) * 1000)


static int w83l785ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id);
static int w83l785ts_detect(struct i2c_client *client,
			    struct i2c_board_info *info);
static int w83l785ts_remove(struct i2c_client *client);
static u8 w83l785ts_read_value(struct i2c_client *client, u8 reg, u8 defval);
static struct w83l785ts_data *w83l785ts_update_device(struct device *dev);

 
static const struct i2c_device_id w83l785ts_id[] = {
	{ "w83l785ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, w83l785ts_id);

static struct i2c_driver w83l785ts_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "w83l785ts",
	},
	.probe		= w83l785ts_probe,
	.remove		= w83l785ts_remove,
	.id_table	= w83l785ts_id,
	.detect		= w83l785ts_detect,
	.address_list	= normal_i2c,
};


struct w83l785ts_data {
	struct device *hwmon_dev;
	struct mutex update_lock;
	char valid; /* zero until following fields are valid */
	unsigned long last_updated; /* in jiffies */

	/* registers values */
	s8 temp[2]; /* 0: input
		       1: critical limit */
};


static ssize_t show_temp(struct device *dev, struct device_attribute *devattr,
	char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct w83l785ts_data *data = w83l785ts_update_device(dev);
	return sprintf(buf, "%d\n", TEMP_FROM_REG(data->temp[attr->index]));
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_max, S_IRUGO, show_temp, NULL, 1);


/* Return 0 if detection is successful, -ENODEV otherwise */
static int w83l785ts_detect(struct i2c_client *client,
			    struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 man_id;
	u8 chip_id;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	/* detection */
	if ((w83l785ts_read_value(client, W83L785TS_REG_CONFIG, 0) & 0x80)
	 || (w83l785ts_read_value(client, W83L785TS_REG_TYPE, 0) & 0xFC)) {
		dev_dbg(&adapter->dev,
			"W83L785TS-S detection failed at 0x%02x\n",
			client->addr);
		return -ENODEV;
	}

	/* Identification */
	man_id = (w83l785ts_read_value(client, W83L785TS_REG_MAN_ID1, 0) << 8)
	       + w83l785ts_read_value(client, W83L785TS_REG_MAN_ID2, 0);
	chip_id = w83l785ts_read_value(client, W83L785TS_REG_CHIP_ID, 0);

	if (man_id != 0x5CA3		/* Winbond */
	 || chip_id != 0x70) {		/* W83L785TS-S */
		dev_dbg(&adapter->dev,
			"Unsupported chip (man_id=0x%04X, chip_id=0x%02X)\n",
			man_id, chip_id);
		return -ENODEV;
	}

	strlcpy(info->type, "w83l785ts", I2C_NAME_SIZE);

	return 0;
}

static int w83l785ts_probe(struct i2c_client *new_client,
			   const struct i2c_device_id *id)
{
	struct w83l785ts_data *data;
	int err = 0;

	data = kzalloc(sizeof(struct w83l785ts_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(new_client, data);
	data->valid = 0;
	mutex_init(&data->update_lock);

	/* Default values in case the first read fails (unlikely). */
	data->temp[1] = data->temp[0] = 0;

	/*
	 * Initialize the W83L785TS chip
	 * Nothing yet, assume it is already started.
	 */

	err = device_create_file(&new_client->dev,
				 &sensor_dev_attr_temp1_input.dev_attr);
	if (err)
		goto exit_remove;

	err = device_create_file(&new_client->dev,
				 &sensor_dev_attr_temp1_max.dev_attr);
	if (err)
		goto exit_remove;

	/* Register sysfs hooks */
	data->hwmon_dev = hwmon_device_register(&new_client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}

	return 0;

exit_remove:
	device_remove_file(&new_client->dev,
			   &sensor_dev_attr_temp1_input.dev_attr);
	device_remove_file(&new_client->dev,
			   &sensor_dev_attr_temp1_max.dev_attr);
	kfree(data);
exit:
	return err;
}

static int w83l785ts_remove(struct i2c_client *client)
{
	struct w83l785ts_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	device_remove_file(&client->dev,
			   &sensor_dev_attr_temp1_input.dev_attr);
	device_remove_file(&client->dev,
			   &sensor_dev_attr_temp1_max.dev_attr);

	kfree(data);
	return 0;
}

static u8 w83l785ts_read_value(struct i2c_client *client, u8 reg, u8 defval)
{
	int value, i;
	struct device *dev;
	const char *prefix;

	/* We might be called during detection, at which point the client
	   isn't yet fully initialized, so we can't use dev_dbg on it */
	if (i2c_get_clientdata(client)) {
		dev = &client->dev;
		prefix = "";
	} else {
		dev = &client->adapter->dev;
		prefix = "w83l785ts: ";
	}

	/* Frequent read errors have been reported on Asus boards, so we
	 * retry on read errors. If it still fails (unlikely), return the
	 * default value requested by the caller. */
	for (i = 1; i <= MAX_RETRIES; i++) {
		value = i2c_smbus_read_byte_data(client, reg);
		if (value >= 0) {
			dev_dbg(dev, "%sRead 0x%02x from register 0x%02x.\n",
				prefix, value, reg);
			return value;
		}
		dev_dbg(dev, "%sRead failed, will retry in %d.\n", prefix, i);
		msleep(i);
	}

	dev_err(dev, "%sCouldn't read value from register 0x%02x.\n", prefix,
		reg);
	return defval;
}

static struct w83l785ts_data *w83l785ts_update_device(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct w83l785ts_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->update_lock);

	if (!data->valid || time_after(jiffies, data->last_updated + HZ * 2)) {
		dev_dbg(&client->dev, "Updating w83l785ts data.\n");
		data->temp[0] = w83l785ts_read_value(client,
				W83L785TS_REG_TEMP, data->temp[0]);
		data->temp[1] = w83l785ts_read_value(client,
				W83L785TS_REG_TEMP_OVER, data->temp[1]);

		data->last_updated = jiffies;
		data->valid = 1;
	}

	mutex_unlock(&data->update_lock);

	return data;
}

static int __init sensors_w83l785ts_init(void)
{
	return i2c_add_driver(&w83l785ts_driver);
}

static void __exit sensors_w83l785ts_exit(void)
{
	i2c_del_driver(&w83l785ts_driver);
}

MODULE_AUTHOR("Jean Delvare <khali@linux-fr.org>");
MODULE_DESCRIPTION("W83L785TS-S driver");
MODULE_LICENSE("GPL");

module_init(sensors_w83l785ts_init);
module_exit(sensors_w83l785ts_exit);
