
#include <linux/string.h>
#include <linux/device.h>
#include <scsi/scsi_host.h>
#include "fnic.h"

static ssize_t fnic_show_state(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct fc_lport *lp = shost_priv(class_to_shost(dev));
	struct fnic *fnic = lport_priv(lp);

	return snprintf(buf, PAGE_SIZE, "%s\n", fnic_state_str[fnic->state]);
}

static ssize_t fnic_show_drv_version(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", DRV_VERSION);
}

static ssize_t fnic_show_link_state(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct fc_lport *lp = shost_priv(class_to_shost(dev));

	return snprintf(buf, PAGE_SIZE, "%s\n", (lp->link_up)
			? "Link Up" : "Link Down");
}

static DEVICE_ATTR(fnic_state, S_IRUGO, fnic_show_state, NULL);
static DEVICE_ATTR(drv_version, S_IRUGO, fnic_show_drv_version, NULL);
static DEVICE_ATTR(link_state, S_IRUGO, fnic_show_link_state, NULL);

struct device_attribute *fnic_attrs[] = {
	&dev_attr_fnic_state,
	&dev_attr_drv_version,
	&dev_attr_link_state,
	NULL,
};
