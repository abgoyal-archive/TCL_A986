

#ifndef _KREF_H_
#define _KREF_H_

#include <linux/types.h>

struct kref {
	atomic_t refcount;
};

void kref_init(struct kref *kref);
void kref_get(struct kref *kref);
int kref_put(struct kref *kref, void (*release) (struct kref *kref));

#endif /* _KREF_H_ */
