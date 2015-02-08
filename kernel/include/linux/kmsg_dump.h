
#ifndef _LINUX_KMSG_DUMP_H
#define _LINUX_KMSG_DUMP_H

#include <linux/list.h>

enum kmsg_dump_reason {
	KMSG_DUMP_OOPS,
	KMSG_DUMP_PANIC,
	KMSG_DUMP_KEXEC,
};

struct kmsg_dumper {
	void (*dump)(struct kmsg_dumper *dumper, enum kmsg_dump_reason reason,
			const char *s1, unsigned long l1,
			const char *s2, unsigned long l2);
	struct list_head list;
	int registered;
};

#ifdef CONFIG_PRINTK
void kmsg_dump(enum kmsg_dump_reason reason);

int kmsg_dump_register(struct kmsg_dumper *dumper);

int kmsg_dump_unregister(struct kmsg_dumper *dumper);
#else
static inline void kmsg_dump(enum kmsg_dump_reason reason)
{
}

static inline int kmsg_dump_register(struct kmsg_dumper *dumper)
{
	return -EINVAL;
}

static inline int kmsg_dump_unregister(struct kmsg_dumper *dumper)
{
	return -EINVAL;
}
#endif

#endif /* _LINUX_KMSG_DUMP_H */
