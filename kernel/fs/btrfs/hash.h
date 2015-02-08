

#ifndef __HASH__
#define __HASH__

#include <linux/crc32c.h>
static inline u64 btrfs_name_hash(const char *name, int len)
{
	return crc32c((u32)~1, name, len);
}
#endif
