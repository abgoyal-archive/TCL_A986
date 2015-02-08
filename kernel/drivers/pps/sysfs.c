


#include <linux/device.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/pps_kernel.h>


static ssize_t pps_show_assert(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pps_device *pps = dev_get_drvdata(dev);

	if (!(pps->info.mode & PPS_CAPTUREASSERT))
		return 0;

	return sprintf(buf, "%lld.%09d#%d\n",
			(long long) pps->assert_tu.sec, pps->assert_tu.nsec,
			pps->assert_sequence);
}

static ssize_t pps_show_clear(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pps_device *pps = dev_get_drvdata(dev);

	if (!(pps->info.mode & PPS_CAPTURECLEAR))
		return 0;

	return sprintf(buf, "%lld.%09d#%d\n",
			(long long) pps->clear_tu.sec, pps->clear_tu.nsec,
			pps->clear_sequence);
}

static ssize_t pps_show_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pps_device *pps = dev_get_drvdata(dev);

	return sprintf(buf, "%4x\n", pps->info.mode);
}

static ssize_t pps_show_echo(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pps_device *pps = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", !!pps->info.echo);
}

static ssize_t pps_show_name(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pps_device *pps = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", pps->info.name);
}

static ssize_t pps_show_path(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pps_device *pps = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", pps->info.path);
}

struct device_attribute pps_attrs[] = {
	__ATTR(assert, S_IRUGO, pps_show_assert, NULL),
	__ATTR(clear, S_IRUGO, pps_show_clear, NULL),
	__ATTR(mode, S_IRUGO, pps_show_mode, NULL),
	__ATTR(echo, S_IRUGO, pps_show_echo, NULL),
	__ATTR(name, S_IRUGO, pps_show_name, NULL),
	__ATTR(path, S_IRUGO, pps_show_path, NULL),
	__ATTR_NULL,
};
