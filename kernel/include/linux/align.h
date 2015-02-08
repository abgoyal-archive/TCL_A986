
#ifndef _LINUX_ALIGN_H
#define _LINUX_ALIGN_H

#define __ALIGN_KERNEL(x, a)	__ALIGN_KERNEL_MASK((x), (typeof(x))(a) - 1)
#define __ALIGN_KERNEL_MASK(x, mask) \
				(((x) + (mask)) & ~(mask))

#ifdef __KERNEL__

#include <linux/types.h>

#define ALIGN(x, a)		__ALIGN_KERNEL((x), (a))
#define __ALIGN_MASK(x, mask)	__ALIGN_KERNEL_MASK((x), (mask))
#define PTR_ALIGN(p, a)		((typeof(p)) ALIGN((unsigned long) (p), (a)))
#define ALIGN_FLOOR(x, a)	__ALIGN_FLOOR_MASK((x), (typeof(x)) (a) - 1)
#define __ALIGN_FLOOR_MASK(x, mask)	((x) & ~(mask))
#define PTR_ALIGN_FLOOR(p, a) \
			((typeof(p)) ALIGN_FLOOR((unsigned long) (p), (a)))
#define IS_ALIGNED(x, a)	(((x) & ((typeof(x)) (a) - 1)) == 0)

#define object_align(obj)	PTR_ALIGN((obj), __alignof__(*(obj)))
#define object_align_floor(obj)	PTR_ALIGN_FLOOR((obj), __alignof__(*(obj)))

static inline size_t offset_align(size_t align_drift, size_t alignment)
{
	return (alignment - align_drift) & (alignment - 1);
}

static inline size_t offset_align_floor(size_t align_drift, size_t alignment)
{
	return (align_drift - alignment) & (alignment - 1);
}

#endif /* __KERNEL__ */

#endif
