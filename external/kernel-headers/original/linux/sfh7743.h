
#ifndef _LINUX_SFH7743_H_
#define _LINUX_SFH7743_H_

#include <linux/ioctl.h>

#ifdef __KERNEL__

struct sfh7743_platform_data {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	int gpio;
} __attribute__ ((packed));

#endif /* __KERNEL__ */

#define SFH7743_IO			0xA2

#define SFH7743_IOCTL_GET_ENABLE	_IOR(SFH7743_IO, 0x00, char)
#define SFH7743_IOCTL_SET_ENABLE	_IOW(SFH7743_IO, 0x01, char)

#endif /* _LINUX_SFH7743_H__ */
