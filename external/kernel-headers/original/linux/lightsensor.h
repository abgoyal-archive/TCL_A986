
#ifndef __LINUX_LIGHTSENSOR_H
#define __LINUX_LIGHTSENSOR_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define LIGHTSENSOR_IOCTL_MAGIC 'l'

#define LIGHTSENSOR_IOCTL_GET_ENABLED _IOR(LIGHTSENSOR_IOCTL_MAGIC, 1, int *)
#define LIGHTSENSOR_IOCTL_ENABLE _IOW(LIGHTSENSOR_IOCTL_MAGIC, 2, int *)

#endif
