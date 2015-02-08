


#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/acpi.h>
#include <linux/io.h>


static unsigned short force_addr;
module_param(force_addr, ushort, 0);
MODULE_PARM_DESC(force_addr,
		 "Initialize the base address of the sensors");

static struct platform_device *pdev;


/* Many VIA686A constants specified below */

/* Length of ISA address segment */
#define VIA686A_EXTENT		0x80
#define VIA686A_BASE_REG	0x70
#define VIA686A_ENABLE_REG	0x74

/* The VIA686A registers */
/* ins numbered 0-4 */
#define VIA686A_REG_IN_MAX(nr)	(0x2b + ((nr) * 2))
#define VIA686A_REG_IN_MIN(nr)	(0x2c + ((nr) * 2))
#define VIA686A_REG_IN(nr)	(0x22 + (nr))

/* fans numbered 1-2 */
#define VIA686A_REG_FAN_MIN(nr)	(0x3a + (nr))
#define VIA686A_REG_FAN(nr)	(0x28 + (nr))

/* temps numbered 1-3 */
static const u8 VIA686A_REG_TEMP[]	= { 0x20, 0x21, 0x1f };
static const u8 VIA686A_REG_TEMP_OVER[]	= { 0x39, 0x3d, 0x1d };
static const u8 VIA686A_REG_TEMP_HYST[]	= { 0x3a, 0x3e, 0x1e };
/* bits 7-6 */
#define VIA686A_REG_TEMP_LOW1	0x4b
/* 2 = bits 5-4, 3 = bits 7-6 */
#define VIA686A_REG_TEMP_LOW23	0x49

#define VIA686A_REG_ALARM1	0x41
#define VIA686A_REG_ALARM2	0x42
#define VIA686A_REG_FANDIV	0x47
#define VIA686A_REG_CONFIG	0x40
#define VIA686A_REG_TEMP_MODE		0x4b
/* We'll just assume that you want to set all 3 simultaneously: */
#define VIA686A_TEMP_MODE_MASK		0x3F
#define VIA686A_TEMP_MODE_CONTINUOUS	0x00

static inline u8 IN_TO_REG(long val, int inNum)
{
	/* To avoid floating point, we multiply constants by 10 (100 for +12V).
	   Rounding is done (120500 is actually 133000 - 12500).
	   Remember that val is expressed in 0.001V/bit, which is why we divide
	   by an additional 10000 (100000 for +12V): 1000 for val and 10 (100)
	   for the constants. */
	if (inNum <= 1)
		return (u8)
		    SENSORS_LIMIT((val * 21024 - 1205000) / 250000, 0, 255);
	else if (inNum == 2)
		return (u8)
		    SENSORS_LIMIT((val * 15737 - 1205000) / 250000, 0, 255);
	else if (inNum == 3)
		return (u8)
		    SENSORS_LIMIT((val * 10108 - 1205000) / 250000, 0, 255);
	else
		return (u8)
		    SENSORS_LIMIT((val * 41714 - 12050000) / 2500000, 0, 255);
}

static inline long IN_FROM_REG(u8 val, int inNum)
{
	/* To avoid floating point, we multiply constants by 10 (100 for +12V).
	   We also multiply them by 1000 because we want 0.001V/bit for the
	   output value. Rounding is done. */
	if (inNum <= 1)
		return (long) ((250000 * val + 1330000 + 21024 / 2) / 21024);
	else if (inNum == 2)
		return (long) ((250000 * val + 1330000 + 15737 / 2) / 15737);
	else if (inNum == 3)
		return (long) ((250000 * val + 1330000 + 10108 / 2) / 10108);
	else
		return (long) ((2500000 * val + 13300000 + 41714 / 2) / 41714);
}

/********* FAN RPM CONVERSIONS ********/
static inline u8 FAN_TO_REG(long rpm, int div)
{
	if (rpm == 0)
		return 0;
	rpm = SENSORS_LIMIT(rpm, 1, 1000000);
	return SENSORS_LIMIT((1350000 + rpm * div / 2) / (rpm * div), 1, 255);
}

#define FAN_FROM_REG(val,div) ((val)==0?0:(val)==255?0:1350000/((val)*(div)))

/******** TEMP CONVERSIONS (Bob Dougherty) *********/
static const s16 tempLUT[] =
{ -709, -688, -667, -646, -627, -607, -589, -570, -553, -536, -519,
	-503, -487, -471, -456, -442, -428, -414, -400, -387, -375,
	-362, -350, -339, -327, -316, -305, -295, -285, -275, -265,
	-255, -246, -237, -229, -220, -212, -204, -196, -188, -180,
	-173, -166, -159, -152, -145, -139, -132, -126, -120, -114,
	-108, -102, -96, -91, -85, -80, -74, -69, -64, -59, -54, -49,
	-44, -39, -34, -29, -25, -20, -15, -11, -6, -2, 3, 7, 12, 16,
	20, 25, 29, 33, 37, 42, 46, 50, 54, 59, 63, 67, 71, 75, 79, 84,
	88, 92, 96, 100, 104, 109, 113, 117, 121, 125, 130, 134, 138,
	142, 146, 151, 155, 159, 163, 168, 172, 176, 181, 185, 189,
	193, 198, 202, 206, 211, 215, 219, 224, 228, 232, 237, 241,
	245, 250, 254, 259, 263, 267, 272, 276, 281, 285, 290, 294,
	299, 303, 307, 312, 316, 321, 325, 330, 334, 339, 344, 348,
	353, 357, 362, 366, 371, 376, 380, 385, 390, 395, 399, 404,
	409, 414, 419, 423, 428, 433, 438, 443, 449, 454, 459, 464,
	469, 475, 480, 486, 491, 497, 502, 508, 514, 520, 526, 532,
	538, 544, 551, 557, 564, 571, 578, 584, 592, 599, 606, 614,
	621, 629, 637, 645, 654, 662, 671, 680, 689, 698, 708, 718,
	728, 738, 749, 759, 770, 782, 793, 805, 818, 830, 843, 856,
	870, 883, 898, 912, 927, 943, 958, 975, 991, 1008, 1026, 1044,
	1062, 1081, 1101, 1121, 1141, 1162, 1184, 1206, 1229, 1252,
	1276, 1301, 1326, 1352, 1378, 1406, 1434, 1462
};

static const u8 viaLUT[] =
{ 12, 12, 13, 14, 14, 15, 16, 16, 17, 18, 18, 19, 20, 20, 21, 22, 23,
	23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 35, 36, 37, 39, 40,
	41, 43, 45, 46, 48, 49, 51, 53, 55, 57, 59, 60, 62, 64, 66,
	69, 71, 73, 75, 77, 79, 82, 84, 86, 88, 91, 93, 95, 98, 100,
	103, 105, 107, 110, 112, 115, 117, 119, 122, 124, 126, 129,
	131, 134, 136, 138, 140, 143, 145, 147, 150, 152, 154, 156,
	158, 160, 162, 164, 166, 168, 170, 172, 174, 176, 178, 180,
	182, 183, 185, 187, 188, 190, 192, 193, 195, 196, 198, 199,
	200, 202, 203, 205, 206, 207, 208, 209, 210, 211, 212, 213,
	214, 215, 216, 217, 218, 219, 220, 221, 222, 222, 223, 224,
	225, 226, 226, 227, 228, 228, 229, 230, 230, 231, 232, 232,
	233, 233, 234, 235, 235, 236, 236, 237, 237, 238, 238, 239,
	239, 240
};

static inline u8 TEMP_TO_REG(long val)
{
	return viaLUT[val <= -50000 ? 0 : val >= 110000 ? 160 :
		      (val < 0 ? val - 500 : val + 500) / 1000 + 50];
}

/* for 8-bit temperature hyst and over registers */
#define TEMP_FROM_REG(val)	((long)tempLUT[val] * 100)

/* for 10-bit temperature readings */
static inline long TEMP_FROM_REG10(u16 val)
{
	u16 eightBits = val >> 2;
	u16 twoBits = val & 3;

	/* no interpolation for these */
	if (twoBits == 0 || eightBits == 255)
		return TEMP_FROM_REG(eightBits);

	/* do some linear interpolation */
	return (tempLUT[eightBits] * (4 - twoBits) +
		tempLUT[eightBits + 1] * twoBits) * 25;
}

#define DIV_FROM_REG(val) (1 << (val))
#define DIV_TO_REG(val) ((val)==8?3:(val)==4?2:(val)==1?0:1)

struct via686a_data {
	unsigned short addr;
	const char *name;
	struct device *hwmon_dev;
	struct mutex update_lock;
	char valid;		/* !=0 if following fields are valid */
	unsigned long last_updated;	/* In jiffies */

	u8 in[5];		/* Register value */
	u8 in_max[5];		/* Register value */
	u8 in_min[5];		/* Register value */
	u8 fan[2];		/* Register value */
	u8 fan_min[2];		/* Register value */
	u16 temp[3];		/* Register value 10 bit */
	u8 temp_over[3];	/* Register value */
	u8 temp_hyst[3];	/* Register value */
	u8 fan_div[2];		/* Register encoding, shifted right */
	u16 alarms;		/* Register encoding, combined */
};

static struct pci_dev *s_bridge;	/* pointer to the (only) via686a */

static int via686a_probe(struct platform_device *pdev);
static int __devexit via686a_remove(struct platform_device *pdev);

static inline int via686a_read_value(struct via686a_data *data, u8 reg)
{
	return inb_p(data->addr + reg);
}

static inline void via686a_write_value(struct via686a_data *data, u8 reg,
				       u8 value)
{
	outb_p(value, data->addr + reg);
}

static struct via686a_data *via686a_update_device(struct device *dev);
static void via686a_init_device(struct via686a_data *data);

/* following are the sysfs callback functions */

/* 7 voltage sensors */
static ssize_t show_in(struct device *dev, struct device_attribute *da,
		char *buf) {
	struct via686a_data *data = via686a_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	return sprintf(buf, "%ld\n", IN_FROM_REG(data->in[nr], nr));
}

static ssize_t show_in_min(struct device *dev, struct device_attribute *da,
		char *buf) {
	struct via686a_data *data = via686a_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	return sprintf(buf, "%ld\n", IN_FROM_REG(data->in_min[nr], nr));
}

static ssize_t show_in_max(struct device *dev, struct device_attribute *da,
		char *buf) {
	struct via686a_data *data = via686a_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	return sprintf(buf, "%ld\n", IN_FROM_REG(data->in_max[nr], nr));
}

static ssize_t set_in_min(struct device *dev, struct device_attribute *da,
		const char *buf, size_t count) {
	struct via686a_data *data = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	unsigned long val = simple_strtoul(buf, NULL, 10);

	mutex_lock(&data->update_lock);
	data->in_min[nr] = IN_TO_REG(val, nr);
	via686a_write_value(data, VIA686A_REG_IN_MIN(nr),
			data->in_min[nr]);
	mutex_unlock(&data->update_lock);
	return count;
}
static ssize_t set_in_max(struct device *dev, struct device_attribute *da,
		const char *buf, size_t count) {
	struct via686a_data *data = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	unsigned long val = simple_strtoul(buf, NULL, 10);

	mutex_lock(&data->update_lock);
	data->in_max[nr] = IN_TO_REG(val, nr);
	via686a_write_value(data, VIA686A_REG_IN_MAX(nr),
			data->in_max[nr]);
	mutex_unlock(&data->update_lock);
	return count;
}
#define show_in_offset(offset)					\
static SENSOR_DEVICE_ATTR(in##offset##_input, S_IRUGO,		\
		show_in, NULL, offset);				\
static SENSOR_DEVICE_ATTR(in##offset##_min, S_IRUGO | S_IWUSR,	\
		show_in_min, set_in_min, offset);		\
static SENSOR_DEVICE_ATTR(in##offset##_max, S_IRUGO | S_IWUSR,	\
		show_in_max, set_in_max, offset);

show_in_offset(0);
show_in_offset(1);
show_in_offset(2);
show_in_offset(3);
show_in_offset(4);

/* 3 temperatures */
static ssize_t show_temp(struct device *dev, struct device_attribute *da,
		char *buf) {
	struct via686a_data *data = via686a_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	return sprintf(buf, "%ld\n", TEMP_FROM_REG10(data->temp[nr]));
}
static ssize_t show_temp_over(struct device *dev, struct device_attribute *da,
		char *buf) {
	struct via686a_data *data = via686a_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	return sprintf(buf, "%ld\n", TEMP_FROM_REG(data->temp_over[nr]));
}
static ssize_t show_temp_hyst(struct device *dev, struct device_attribute *da,
		char *buf) {
	struct via686a_data *data = via686a_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	return sprintf(buf, "%ld\n", TEMP_FROM_REG(data->temp_hyst[nr]));
}
static ssize_t set_temp_over(struct device *dev, struct device_attribute *da,
		const char *buf, size_t count) {
	struct via686a_data *data = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	int val = simple_strtol(buf, NULL, 10);

	mutex_lock(&data->update_lock);
	data->temp_over[nr] = TEMP_TO_REG(val);
	via686a_write_value(data, VIA686A_REG_TEMP_OVER[nr],
			    data->temp_over[nr]);
	mutex_unlock(&data->update_lock);
	return count;
}
static ssize_t set_temp_hyst(struct device *dev, struct device_attribute *da,
		const char *buf, size_t count) {
	struct via686a_data *data = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	int val = simple_strtol(buf, NULL, 10);

	mutex_lock(&data->update_lock);
	data->temp_hyst[nr] = TEMP_TO_REG(val);
	via686a_write_value(data, VIA686A_REG_TEMP_HYST[nr],
			    data->temp_hyst[nr]);
	mutex_unlock(&data->update_lock);
	return count;
}
#define show_temp_offset(offset)					\
static SENSOR_DEVICE_ATTR(temp##offset##_input, S_IRUGO,		\
		show_temp, NULL, offset - 1);				\
static SENSOR_DEVICE_ATTR(temp##offset##_max, S_IRUGO | S_IWUSR,	\
		show_temp_over, set_temp_over, offset - 1);		\
static SENSOR_DEVICE_ATTR(temp##offset##_max_hyst, S_IRUGO | S_IWUSR,	\
		show_temp_hyst, set_temp_hyst, offset - 1);

show_temp_offset(1);
show_temp_offset(2);
show_temp_offset(3);

/* 2 Fans */
static ssize_t show_fan(struct device *dev, struct device_attribute *da,
		char *buf) {
	struct via686a_data *data = via686a_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	return sprintf(buf, "%d\n", FAN_FROM_REG(data->fan[nr],
				DIV_FROM_REG(data->fan_div[nr])) );
}
static ssize_t show_fan_min(struct device *dev, struct device_attribute *da,
		char *buf) {
	struct via686a_data *data = via686a_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	return sprintf(buf, "%d\n",
		FAN_FROM_REG(data->fan_min[nr], DIV_FROM_REG(data->fan_div[nr])) );
}
static ssize_t show_fan_div(struct device *dev, struct device_attribute *da,
		char *buf) {
	struct via686a_data *data = via686a_update_device(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	return sprintf(buf, "%d\n", DIV_FROM_REG(data->fan_div[nr]) );
}
static ssize_t set_fan_min(struct device *dev, struct device_attribute *da,
		const char *buf, size_t count) {
	struct via686a_data *data = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	int val = simple_strtol(buf, NULL, 10);

	mutex_lock(&data->update_lock);
	data->fan_min[nr] = FAN_TO_REG(val, DIV_FROM_REG(data->fan_div[nr]));
	via686a_write_value(data, VIA686A_REG_FAN_MIN(nr+1), data->fan_min[nr]);
	mutex_unlock(&data->update_lock);
	return count;
}
static ssize_t set_fan_div(struct device *dev, struct device_attribute *da,
		const char *buf, size_t count) {
	struct via686a_data *data = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int nr = attr->index;
	int val = simple_strtol(buf, NULL, 10);
	int old;

	mutex_lock(&data->update_lock);
	old = via686a_read_value(data, VIA686A_REG_FANDIV);
	data->fan_div[nr] = DIV_TO_REG(val);
	old = (old & 0x0f) | (data->fan_div[1] << 6) | (data->fan_div[0] << 4);
	via686a_write_value(data, VIA686A_REG_FANDIV, old);
	mutex_unlock(&data->update_lock);
	return count;
}

#define show_fan_offset(offset)						\
static SENSOR_DEVICE_ATTR(fan##offset##_input, S_IRUGO,			\
		show_fan, NULL, offset - 1);				\
static SENSOR_DEVICE_ATTR(fan##offset##_min, S_IRUGO | S_IWUSR,		\
		show_fan_min, set_fan_min, offset - 1);			\
static SENSOR_DEVICE_ATTR(fan##offset##_div, S_IRUGO | S_IWUSR,		\
		show_fan_div, set_fan_div, offset - 1);

show_fan_offset(1);
show_fan_offset(2);

/* Alarms */
static ssize_t show_alarms(struct device *dev, struct device_attribute *attr, char *buf) {
	struct via686a_data *data = via686a_update_device(dev);
	return sprintf(buf, "%u\n", data->alarms);
}
static DEVICE_ATTR(alarms, S_IRUGO, show_alarms, NULL);

static ssize_t show_alarm(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int bitnr = to_sensor_dev_attr(attr)->index;
	struct via686a_data *data = via686a_update_device(dev);
	return sprintf(buf, "%u\n", (data->alarms >> bitnr) & 1);
}
static SENSOR_DEVICE_ATTR(in0_alarm, S_IRUGO, show_alarm, NULL, 0);
static SENSOR_DEVICE_ATTR(in1_alarm, S_IRUGO, show_alarm, NULL, 1);
static SENSOR_DEVICE_ATTR(in2_alarm, S_IRUGO, show_alarm, NULL, 2);
static SENSOR_DEVICE_ATTR(in3_alarm, S_IRUGO, show_alarm, NULL, 3);
static SENSOR_DEVICE_ATTR(in4_alarm, S_IRUGO, show_alarm, NULL, 8);
static SENSOR_DEVICE_ATTR(temp1_alarm, S_IRUGO, show_alarm, NULL, 4);
static SENSOR_DEVICE_ATTR(temp2_alarm, S_IRUGO, show_alarm, NULL, 11);
static SENSOR_DEVICE_ATTR(temp3_alarm, S_IRUGO, show_alarm, NULL, 15);
static SENSOR_DEVICE_ATTR(fan1_alarm, S_IRUGO, show_alarm, NULL, 6);
static SENSOR_DEVICE_ATTR(fan2_alarm, S_IRUGO, show_alarm, NULL, 7);

static ssize_t show_name(struct device *dev, struct device_attribute
			 *devattr, char *buf)
{
	struct via686a_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", data->name);
}
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

static struct attribute *via686a_attributes[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in0_min.dev_attr.attr,
	&sensor_dev_attr_in1_min.dev_attr.attr,
	&sensor_dev_attr_in2_min.dev_attr.attr,
	&sensor_dev_attr_in3_min.dev_attr.attr,
	&sensor_dev_attr_in4_min.dev_attr.attr,
	&sensor_dev_attr_in0_max.dev_attr.attr,
	&sensor_dev_attr_in1_max.dev_attr.attr,
	&sensor_dev_attr_in2_max.dev_attr.attr,
	&sensor_dev_attr_in3_max.dev_attr.attr,
	&sensor_dev_attr_in4_max.dev_attr.attr,
	&sensor_dev_attr_in0_alarm.dev_attr.attr,
	&sensor_dev_attr_in1_alarm.dev_attr.attr,
	&sensor_dev_attr_in2_alarm.dev_attr.attr,
	&sensor_dev_attr_in3_alarm.dev_attr.attr,
	&sensor_dev_attr_in4_alarm.dev_attr.attr,

	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp3_input.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp2_max.dev_attr.attr,
	&sensor_dev_attr_temp3_max.dev_attr.attr,
	&sensor_dev_attr_temp1_max_hyst.dev_attr.attr,
	&sensor_dev_attr_temp2_max_hyst.dev_attr.attr,
	&sensor_dev_attr_temp3_max_hyst.dev_attr.attr,
	&sensor_dev_attr_temp1_alarm.dev_attr.attr,
	&sensor_dev_attr_temp2_alarm.dev_attr.attr,
	&sensor_dev_attr_temp3_alarm.dev_attr.attr,

	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan1_min.dev_attr.attr,
	&sensor_dev_attr_fan2_min.dev_attr.attr,
	&sensor_dev_attr_fan1_div.dev_attr.attr,
	&sensor_dev_attr_fan2_div.dev_attr.attr,
	&sensor_dev_attr_fan1_alarm.dev_attr.attr,
	&sensor_dev_attr_fan2_alarm.dev_attr.attr,

	&dev_attr_alarms.attr,
	&dev_attr_name.attr,
	NULL
};

static const struct attribute_group via686a_group = {
	.attrs = via686a_attributes,
};

static struct platform_driver via686a_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "via686a",
	},
	.probe		= via686a_probe,
	.remove		= __devexit_p(via686a_remove),
};


/* This is called when the module is loaded */
static int __devinit via686a_probe(struct platform_device *pdev)
{
	struct via686a_data *data;
	struct resource *res;
	int err;

	/* Reserve the ISA region */
	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!request_region(res->start, VIA686A_EXTENT,
			    via686a_driver.driver.name)) {
		dev_err(&pdev->dev, "Region 0x%lx-0x%lx already in use!\n",
			(unsigned long)res->start, (unsigned long)res->end);
		return -ENODEV;
	}

	if (!(data = kzalloc(sizeof(struct via686a_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit_release;
	}

	platform_set_drvdata(pdev, data);
	data->addr = res->start;
	data->name = "via686a";
	mutex_init(&data->update_lock);

	/* Initialize the VIA686A chip */
	via686a_init_device(data);

	/* Register sysfs hooks */
	if ((err = sysfs_create_group(&pdev->dev.kobj, &via686a_group)))
		goto exit_free;

	data->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove_files;
	}

	return 0;

exit_remove_files:
	sysfs_remove_group(&pdev->dev.kobj, &via686a_group);
exit_free:
	kfree(data);
exit_release:
	release_region(res->start, VIA686A_EXTENT);
	return err;
}

static int __devexit via686a_remove(struct platform_device *pdev)
{
	struct via686a_data *data = platform_get_drvdata(pdev);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&pdev->dev.kobj, &via686a_group);

	release_region(data->addr, VIA686A_EXTENT);
	platform_set_drvdata(pdev, NULL);
	kfree(data);

	return 0;
}

static void __devinit via686a_init_device(struct via686a_data *data)
{
	u8 reg;

	/* Start monitoring */
	reg = via686a_read_value(data, VIA686A_REG_CONFIG);
	via686a_write_value(data, VIA686A_REG_CONFIG, (reg | 0x01) & 0x7F);

	/* Configure temp interrupt mode for continuous-interrupt operation */
	reg = via686a_read_value(data, VIA686A_REG_TEMP_MODE);
	via686a_write_value(data, VIA686A_REG_TEMP_MODE,
			    (reg & ~VIA686A_TEMP_MODE_MASK)
			    | VIA686A_TEMP_MODE_CONTINUOUS);
}

static struct via686a_data *via686a_update_device(struct device *dev)
{
	struct via686a_data *data = dev_get_drvdata(dev);
	int i;

	mutex_lock(&data->update_lock);

	if (time_after(jiffies, data->last_updated + HZ + HZ / 2)
	    || !data->valid) {
		for (i = 0; i <= 4; i++) {
			data->in[i] =
			    via686a_read_value(data, VIA686A_REG_IN(i));
			data->in_min[i] = via686a_read_value(data,
							     VIA686A_REG_IN_MIN
							     (i));
			data->in_max[i] =
			    via686a_read_value(data, VIA686A_REG_IN_MAX(i));
		}
		for (i = 1; i <= 2; i++) {
			data->fan[i - 1] =
			    via686a_read_value(data, VIA686A_REG_FAN(i));
			data->fan_min[i - 1] = via686a_read_value(data,
						     VIA686A_REG_FAN_MIN(i));
		}
		for (i = 0; i <= 2; i++) {
			data->temp[i] = via686a_read_value(data,
						 VIA686A_REG_TEMP[i]) << 2;
			data->temp_over[i] =
			    via686a_read_value(data,
					       VIA686A_REG_TEMP_OVER[i]);
			data->temp_hyst[i] =
			    via686a_read_value(data,
					       VIA686A_REG_TEMP_HYST[i]);
		}
		/* add in lower 2 bits
		   temp1 uses bits 7-6 of VIA686A_REG_TEMP_LOW1
		   temp2 uses bits 5-4 of VIA686A_REG_TEMP_LOW23
		   temp3 uses bits 7-6 of VIA686A_REG_TEMP_LOW23
		 */
		data->temp[0] |= (via686a_read_value(data,
						     VIA686A_REG_TEMP_LOW1)
				  & 0xc0) >> 6;
		data->temp[1] |=
		    (via686a_read_value(data, VIA686A_REG_TEMP_LOW23) &
		     0x30) >> 4;
		data->temp[2] |=
		    (via686a_read_value(data, VIA686A_REG_TEMP_LOW23) &
		     0xc0) >> 6;

		i = via686a_read_value(data, VIA686A_REG_FANDIV);
		data->fan_div[0] = (i >> 4) & 0x03;
		data->fan_div[1] = i >> 6;
		data->alarms =
		    via686a_read_value(data,
				       VIA686A_REG_ALARM1) |
		    (via686a_read_value(data, VIA686A_REG_ALARM2) << 8);
		data->last_updated = jiffies;
		data->valid = 1;
	}

	mutex_unlock(&data->update_lock);

	return data;
}

static const struct pci_device_id via686a_pci_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA_82C686_4) },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, via686a_pci_ids);

static int __devinit via686a_device_add(unsigned short address)
{
	struct resource res = {
		.start	= address,
		.end	= address + VIA686A_EXTENT - 1,
		.name	= "via686a",
		.flags	= IORESOURCE_IO,
	};
	int err;

	err = acpi_check_resource_conflict(&res);
	if (err)
		goto exit;

	pdev = platform_device_alloc("via686a", address);
	if (!pdev) {
		err = -ENOMEM;
		printk(KERN_ERR "via686a: Device allocation failed\n");
		goto exit;
	}

	err = platform_device_add_resources(pdev, &res, 1);
	if (err) {
		printk(KERN_ERR "via686a: Device resource addition failed "
		       "(%d)\n", err);
		goto exit_device_put;
	}

	err = platform_device_add(pdev);
	if (err) {
		printk(KERN_ERR "via686a: Device addition failed (%d)\n",
		       err);
		goto exit_device_put;
	}

	return 0;

exit_device_put:
	platform_device_put(pdev);
exit:
	return err;
}

static int __devinit via686a_pci_probe(struct pci_dev *dev,
				       const struct pci_device_id *id)
{
	u16 address, val;

	if (force_addr) {
		address = force_addr & ~(VIA686A_EXTENT - 1);
		dev_warn(&dev->dev, "Forcing ISA address 0x%x\n", address);
		if (PCIBIOS_SUCCESSFUL !=
		    pci_write_config_word(dev, VIA686A_BASE_REG, address | 1))
			return -ENODEV;
	}
	if (PCIBIOS_SUCCESSFUL !=
	    pci_read_config_word(dev, VIA686A_BASE_REG, &val))
		return -ENODEV;

	address = val & ~(VIA686A_EXTENT - 1);
	if (address == 0) {
		dev_err(&dev->dev, "base address not set - upgrade BIOS "
			"or use force_addr=0xaddr\n");
		return -ENODEV;
	}

	if (PCIBIOS_SUCCESSFUL !=
	    pci_read_config_word(dev, VIA686A_ENABLE_REG, &val))
		return -ENODEV;
	if (!(val & 0x0001)) {
		if (!force_addr) {
			dev_warn(&dev->dev, "Sensors disabled, enable "
				 "with force_addr=0x%x\n", address);
			return -ENODEV;
		}

		dev_warn(&dev->dev, "Enabling sensors\n");
		if (PCIBIOS_SUCCESSFUL !=
		    pci_write_config_word(dev, VIA686A_ENABLE_REG,
					  val | 0x0001))
			return -ENODEV;
	}

	if (platform_driver_register(&via686a_driver))
		goto exit;

	/* Sets global pdev as a side effect */
	if (via686a_device_add(address))
		goto exit_unregister;

	/* Always return failure here.  This is to allow other drivers to bind
	 * to this pci device.  We don't really want to have control over the
	 * pci device, we only wanted to read as few register values from it.
	 */
	s_bridge = pci_dev_get(dev);
	return -ENODEV;

exit_unregister:
	platform_driver_unregister(&via686a_driver);
exit:
	return -ENODEV;
}

static struct pci_driver via686a_pci_driver = {
	.name		= "via686a",
	.id_table	= via686a_pci_ids,
	.probe		= via686a_pci_probe,
};

static int __init sm_via686a_init(void)
{
	return pci_register_driver(&via686a_pci_driver);
}

static void __exit sm_via686a_exit(void)
{
	pci_unregister_driver(&via686a_pci_driver);
	if (s_bridge != NULL) {
		platform_device_unregister(pdev);
		platform_driver_unregister(&via686a_driver);
		pci_dev_put(s_bridge);
		s_bridge = NULL;
	}
}

MODULE_AUTHOR("Kyösti Mälkki <kmalkki@cc.hut.fi>, "
	      "Mark Studebaker <mdsxyz123@yahoo.com> "
	      "and Bob Dougherty <bobd@stanford.edu>");
MODULE_DESCRIPTION("VIA 686A Sensor device");
MODULE_LICENSE("GPL");

module_init(sm_via686a_init);
module_exit(sm_via686a_exit);
