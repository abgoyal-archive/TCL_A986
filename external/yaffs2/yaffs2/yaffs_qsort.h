
#ifndef QSORT_H
#define QSORT_H

extern void qsort (void *const pbase,
		    size_t total_elems,
		    size_t size,
		    int (*cmp)(const void *, const void *));

#endif
