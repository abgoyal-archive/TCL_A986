

#ifndef __RTMP_OS_H__
#define __RTMP_OS_H__

#ifdef LINUX
#include "rt_linux.h"
#endif /* LINUX // */

struct rt_rtmp_os_netdev_op_hook {
	const struct net_device_ops *netdev_ops;
	void *priv;
	int priv_flags;
	unsigned char devAddr[6];
	unsigned char devName[16];
	unsigned char needProtcted;
};

typedef enum _RTMP_TASK_STATUS_ {
	RTMP_TASK_STAT_UNKNOWN = 0,
	RTMP_TASK_STAT_INITED = 1,
	RTMP_TASK_STAT_RUNNING = 2,
	RTMP_TASK_STAT_STOPED = 4,
} RTMP_TASK_STATUS;
#define RTMP_TASK_CAN_DO_INSERT		(RTMP_TASK_STAT_INITED |RTMP_TASK_STAT_RUNNING)

#define RTMP_OS_TASK_NAME_LEN	16
struct rt_rtmp_os_task {
	char taskName[RTMP_OS_TASK_NAME_LEN];
	void *priv;
	/*unsigned long         taskFlags; */
	RTMP_TASK_STATUS taskStatus;
#ifndef KTHREAD_SUPPORT
	struct semaphore taskSema;
	struct pid *taskPID;
	struct completion taskComplete;
#endif
	unsigned char task_killed;
#ifdef KTHREAD_SUPPORT
	struct task_struct *kthread_task;
	wait_queue_head_t kthread_q;
	BOOLEAN kthread_running;
#endif
};

int RtmpOSIRQRequest(struct net_device *pNetDev);
int RtmpOSIRQRelease(struct net_device *pNetDev);

#endif /* __RMTP_OS_H__ // */
