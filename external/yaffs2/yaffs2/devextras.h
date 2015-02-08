
#ifndef __EXTRAS_H__
#define __EXTRAS_H__

#if defined WIN32
#define __inline__ __inline
#define new newHack
#endif

#if !(defined __KERNEL__) || (defined WIN32)

/* User space defines */

typedef unsigned char __u8;
typedef unsigned short __u16;
typedef unsigned __u32;

#if defined(__APPLE__) || defined(__FreeBSD__)
typedef long long loff_t;
#endif


#define prefetch(x) 1

struct list_head {
	struct list_head *next, *prev;
};

#define LIST_HEAD_INIT(name) { &(name), &(name) }

#define LIST_HEAD(name) \
	struct list_head name = LIST_HEAD_INIT(name)

#define INIT_LIST_HEAD(ptr) do { \
	(ptr)->next = (ptr); (ptr)->prev = (ptr); \
} while (0)

static __inline__ void __list_add(struct list_head *new,
				  struct list_head *prev,
				  struct list_head *next)
{
	next->prev = new;
	new->next = next;
	new->prev = prev;
	prev->next = new;
}

static __inline__ void list_add(struct list_head *new, struct list_head *head)
{
	__list_add(new, head, head->next);
}

static __inline__ void list_add_tail(struct list_head *new,
				     struct list_head *head)
{
	__list_add(new, head->prev, head);
}

static __inline__ void __list_del(struct list_head *prev,
				  struct list_head *next)
{
	next->prev = prev;
	prev->next = next;
}

static __inline__ void list_del(struct list_head *entry)
{
	__list_del(entry->prev, entry->next);
}

static __inline__ void list_del_init(struct list_head *entry)
{
	__list_del(entry->prev, entry->next);
	INIT_LIST_HEAD(entry);
}

static __inline__ int list_empty(struct list_head *head)
{
	return head->next == head;
}

static __inline__ void list_splice(struct list_head *list,
				   struct list_head *head)
{
	struct list_head *first = list->next;

	if (first != list) {
		struct list_head *last = list->prev;
		struct list_head *at = head->next;

		first->prev = head;
		head->next = first;

		last->next = at;
		at->prev = last;
	}
}

#define list_entry(ptr, type, member) \
	((type *)((char *)(ptr)-(unsigned long)(&((type *)0)->member)))

#define list_for_each(pos, head) \
	for (pos = (head)->next, prefetch(pos->next); pos != (head); \
        	pos = pos->next, prefetch(pos->next))

#define list_for_each_safe(pos, n, head) \
	for (pos = (head)->next, n = pos->next; pos != (head); \
		pos = n, n = pos->next)

#define DT_UNKNOWN	0
#define DT_FIFO		1
#define DT_CHR		2
#define DT_DIR		4
#define DT_BLK		6
#define DT_REG		8
#define DT_LNK		10
#define DT_SOCK		12
#define DT_WHT		14

#ifndef WIN32
#include <sys/stat.h>
#endif

#define ATTR_MODE	1
#define ATTR_UID	2
#define ATTR_GID	4
#define ATTR_SIZE	8
#define ATTR_ATIME	16
#define ATTR_MTIME	32
#define ATTR_CTIME	64
#define ATTR_ATIME_SET	128
#define ATTR_MTIME_SET	256
#define ATTR_FORCE	512	/* Not a change, but a change it */
#define ATTR_ATTR_FLAG	1024

struct iattr {
	unsigned int ia_valid;
	unsigned ia_mode;
	unsigned ia_uid;
	unsigned ia_gid;
	unsigned ia_size;
	unsigned ia_atime;
	unsigned ia_mtime;
	unsigned ia_ctime;
	unsigned int ia_attr_flags;
};

#define KERN_DEBUG

#else

#ifndef WIN32
#include <linux/types.h>
#include <linux/list.h>
#include <linux/fs.h>
#include <linux/stat.h>
#endif

#endif

#if defined WIN32
#undef new
#endif

#endif
