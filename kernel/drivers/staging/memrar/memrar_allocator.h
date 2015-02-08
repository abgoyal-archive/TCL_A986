

#ifndef MEMRAR_ALLOCATOR_H
#define MEMRAR_ALLOCATOR_H


#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/kernel.h>


struct memrar_address_range {
/* private: internal use only */
	unsigned long begin;
	unsigned long end;
};

struct memrar_address_ranges {
/* private: internal use only */
	struct list_head list;
	struct memrar_address_range range;
};

struct memrar_allocator {
/* private: internal use only */
	struct mutex lock;
	unsigned long base;
	size_t capacity;
	size_t block_size;
	size_t largest_free_area;
	struct memrar_address_ranges allocated_list;
	struct memrar_address_ranges free_list;
};

struct memrar_allocator *memrar_create_allocator(unsigned long base,
						 size_t capacity,
						 size_t block_size);

void memrar_destroy_allocator(struct memrar_allocator *allocator);

unsigned long memrar_allocator_alloc(struct memrar_allocator *allocator,
				     size_t size);

long memrar_allocator_free(struct memrar_allocator *allocator,
			   unsigned long address);

#endif  /* MEMRAR_ALLOCATOR_H */


