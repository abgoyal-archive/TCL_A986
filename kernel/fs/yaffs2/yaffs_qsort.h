


#ifndef __YAFFS_QSORT_H__
#define __YAFFS_QSORT_H__

#ifdef __KERNEL__
#include <linux/sort.h>

extern void yaffs_qsort(void *const base, size_t total_elems, size_t size,
			int (*cmp)(const void *, const void *)){
	sort(base, total_elems, size, cmp, NULL);
}

#else

extern void yaffs_qsort(void *const base, size_t total_elems, size_t size,
			int (*cmp)(const void *, const void *));

#endif
#endif
