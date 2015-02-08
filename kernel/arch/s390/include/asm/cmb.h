
#ifndef S390_CMB_H
#define S390_CMB_H

#include <linux/types.h>

struct cmbdata {
	__u64 size;
	__u64 elapsed_time;
 /* basic and exended format: */
	__u64 ssch_rsch_count;
	__u64 sample_count;
	__u64 device_connect_time;
	__u64 function_pending_time;
	__u64 device_disconnect_time;
	__u64 control_unit_queuing_time;
	__u64 device_active_only_time;
 /* extended format only: */
	__u64 device_busy_time;
	__u64 initial_command_response_time;
};

/* enable channel measurement */
#define BIODASDCMFENABLE	_IO(DASD_IOCTL_LETTER, 32)
/* enable channel measurement */
#define BIODASDCMFDISABLE	_IO(DASD_IOCTL_LETTER, 33)
/* read channel measurement data */
#define BIODASDREADALLCMB	_IOWR(DASD_IOCTL_LETTER, 33, struct cmbdata)

#ifdef __KERNEL__
struct ccw_device;
extern int enable_cmf(struct ccw_device *cdev);
extern int disable_cmf(struct ccw_device *cdev);
extern u64 cmf_read(struct ccw_device *cdev, int index);
extern int cmf_readall(struct ccw_device *cdev, struct cmbdata *data);

#endif /* __KERNEL__ */
#endif /* S390_CMB_H */
