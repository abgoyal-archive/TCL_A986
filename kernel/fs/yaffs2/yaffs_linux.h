

#ifndef __YAFFS_LINUX_H__
#define __YAFFS_LINUX_H__

#include "devextras.h"
#include "yportenv.h"

struct yaffs_LinuxContext {
	struct ylist_head	contextList; /* List of these we have mounted */
	struct yaffs_DeviceStruct *dev;
	struct super_block * superBlock;
	struct task_struct *bgThread; /* Background thread for this device */
	int bgRunning;
        struct semaphore grossLock;     /* Gross locking semaphore */
	__u8 *spareBuffer;      /* For mtdif2 use. Don't know the size of the buffer
				 * at compile time so we have to allocate it.
				 */
	struct ylist_head searchContexts;
	void (*putSuperFunc)(struct super_block *sb);

	struct task_struct *readdirProcess;
	unsigned mount_id;
};

#define yaffs_DeviceToLC(dev) ((struct yaffs_LinuxContext *)((dev)->osContext))
#define yaffs_DeviceToMtd(dev) ((struct mtd_info *)((dev)->driverContext))

#endif

