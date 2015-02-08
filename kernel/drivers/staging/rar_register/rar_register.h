


#ifndef _RAR_REGISTER_H
#define _RAR_REGISTER_H

#include <linux/types.h>

/* following are used both in drivers as well as user space apps */

#define	RAR_TYPE_VIDEO	0
#define	RAR_TYPE_AUDIO	1
#define	RAR_TYPE_IMAGE	2
#define	RAR_TYPE_DATA	3

#ifdef __KERNEL__

struct rar_device;

int register_rar(int num,
		int (*callback)(unsigned long data), unsigned long data);
void unregister_rar(int num);
int rar_get_address(int rar_index, dma_addr_t *start, dma_addr_t *end);
int rar_lock(int rar_index);

#endif  /* __KERNEL__ */
#endif  /* _RAR_REGISTER_H */
