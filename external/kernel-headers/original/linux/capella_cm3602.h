
#ifndef __LINUX_CAPELLA_CM3602_H
#define __LINUX_CAPELLA_CM3602_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define CAPELLA_CM3602_IOCTL_MAGIC 'c'
#define CAPELLA_CM3602_IOCTL_GET_ENABLED \
		_IOR(CAPELLA_CM3602_IOCTL_MAGIC, 1, int *)
#define CAPELLA_CM3602_IOCTL_ENABLE \
		_IOW(CAPELLA_CM3602_IOCTL_MAGIC, 2, int *)

#ifdef __KERNEL__
#define CAPELLA_CM3602 "capella_cm3602"
struct capella_cm3602_platform_data {
	int (*power)(int); /* power to the chip */
	int p_en; /* proximity-sensor enable */
	int p_out; /* proximity-sensor outpuCAPELLA_CM3602_IOCTL_ENABLE,t */
};
#endif /* __KERNEL__ */

#endif
