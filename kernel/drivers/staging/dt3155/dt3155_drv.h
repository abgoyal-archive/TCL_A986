

#ifndef DT3155_DRV_INC
#define DT3155_DRV_INC

/* kernel logical address of the frame grabbers */
extern u8 *dt3155_lbase[MAXBOARDS];

/* kernel logical address of ram buffer */
extern u8 *dt3155_bbase;

#ifdef __KERNEL__
#include <linux/wait.h>

/* wait queue for reads */
extern wait_queue_head_t dt3155_read_wait_queue[MAXBOARDS];
#endif

/* number of devices */
extern u32 ndevices;

extern int dt3155_errno;

#endif
