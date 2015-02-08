
#ifndef _LINUX_FIRMWARE_MAP_H
#define _LINUX_FIRMWARE_MAP_H

#include <linux/list.h>
#include <linux/kobject.h>

#ifdef CONFIG_FIRMWARE_MEMMAP

int firmware_map_add_early(u64 start, u64 end, const char *type);
int firmware_map_add_hotplug(u64 start, u64 end, const char *type);

#else /* CONFIG_FIRMWARE_MEMMAP */

static inline int firmware_map_add_early(u64 start, u64 end, const char *type)
{
	return 0;
}

static inline int firmware_map_add_hotplug(u64 start, u64 end, const char *type)
{
	return 0;
}

#endif /* CONFIG_FIRMWARE_MEMMAP */

#endif /* _LINUX_FIRMWARE_MAP_H */
