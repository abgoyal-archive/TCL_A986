


#ifndef __V4L2_I2C_DRV_H__
#define __V4L2_I2C_DRV_H__

#include <media/v4l2-common.h>

struct v4l2_i2c_driver_data {
	const char * const name;
	int (*command)(struct i2c_client *client, unsigned int cmd, void *arg);
	int (*probe)(struct i2c_client *client, const struct i2c_device_id *id);
	int (*remove)(struct i2c_client *client);
	int (*suspend)(struct i2c_client *client, pm_message_t state);
	int (*resume)(struct i2c_client *client);
	const struct i2c_device_id *id_table;
};

static struct v4l2_i2c_driver_data v4l2_i2c_data;
static struct i2c_driver v4l2_i2c_driver;


/* Bus-based I2C implementation for kernels >= 2.6.26 */

static int __init v4l2_i2c_drv_init(void)
{
	v4l2_i2c_driver.driver.name = v4l2_i2c_data.name;
	v4l2_i2c_driver.command = v4l2_i2c_data.command;
	v4l2_i2c_driver.probe = v4l2_i2c_data.probe;
	v4l2_i2c_driver.remove = v4l2_i2c_data.remove;
	v4l2_i2c_driver.suspend = v4l2_i2c_data.suspend;
	v4l2_i2c_driver.resume = v4l2_i2c_data.resume;
	v4l2_i2c_driver.id_table = v4l2_i2c_data.id_table;
	return i2c_add_driver(&v4l2_i2c_driver);
}


static void __exit v4l2_i2c_drv_cleanup(void)
{
	i2c_del_driver(&v4l2_i2c_driver);
}

module_init(v4l2_i2c_drv_init);
module_exit(v4l2_i2c_drv_cleanup);

#endif /* __V4L2_I2C_DRV_H__ */
