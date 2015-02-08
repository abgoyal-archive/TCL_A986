


#ifndef _MEMRAR_H
#define _MEMRAR_H

#include <linux/ioctl.h>
#include <linux/types.h>


struct RAR_stat {
	__u32 type;
	__u32 capacity;
	__u32 largest_block_size;
};


struct RAR_block_info {
	__u32 type;
	__u32 size;
	__u32 handle;
};


#define RAR_IOCTL_BASE 0xE0

/* Reserve RAR block. */
#define RAR_HANDLER_RESERVE _IOWR(RAR_IOCTL_BASE, 0x00, struct RAR_block_info)

/* Release previously reserved RAR block. */
#define RAR_HANDLER_RELEASE _IOW(RAR_IOCTL_BASE, 0x01, __u32)

/* Get RAR stats. */
#define RAR_HANDLER_STAT    _IOWR(RAR_IOCTL_BASE, 0x02, struct RAR_stat)


#ifdef __KERNEL__

/* -------------------------------------------------------------- */
/*               Kernel Side RAR Handler Interface                */
/* -------------------------------------------------------------- */

struct RAR_buffer {
	struct RAR_block_info info;
	dma_addr_t bus_address;
};

extern size_t rar_reserve(struct RAR_buffer *buffers,
			  size_t count);

extern size_t rar_release(struct RAR_buffer *buffers,
			  size_t count);

extern size_t rar_handle_to_bus(struct RAR_buffer *buffers,
				size_t count);


#endif  /* __KERNEL__ */

#endif  /* _MEMRAR_H */
