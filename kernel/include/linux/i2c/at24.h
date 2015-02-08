
#ifndef _LINUX_AT24_H
#define _LINUX_AT24_H

#include <linux/types.h>
#include <linux/memory.h>


struct at24_platform_data {
	u32		byte_len;		/* size (sum of all addr) */
	u16		page_size;		/* for writes */
	u8		flags;
#define AT24_FLAG_ADDR16	0x80	/* address pointer is 16 bit */
#define AT24_FLAG_READONLY	0x40	/* sysfs-entry will be read-only */
#define AT24_FLAG_IRUGO		0x20	/* sysfs-entry will be world-readable */
#define AT24_FLAG_TAKE8ADDR	0x10	/* take always 8 addresses (24c00) */

	void		(*setup)(struct memory_accessor *, void *context);
	void		*context;
};

#endif /* _LINUX_AT24_H */
