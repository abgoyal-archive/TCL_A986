

#ifndef DRM_SMAN_H
#define DRM_SMAN_H

#include "drmP.h"
#include "drm_hashtab.h"


struct drm_sman_mm {
	/* private info. If allocated, needs to be destroyed by the destroy
	   function */
	void *private;

	/* Allocate a memory block with given size and alignment.
	   Return an opaque reference to the memory block */

	void *(*allocate) (void *private, unsigned long size,
			   unsigned alignment);

	/* Free a memory block. "ref" is the opaque reference that we got from
	   the "alloc" function */

	void (*free) (void *private, void *ref);

	/* Free all resources associated with this allocator */

	void (*destroy) (void *private);

	/* Return a memory offset from the opaque reference returned from the
	   "alloc" function */

	unsigned long (*offset) (void *private, void *ref);
};

struct drm_memblock_item {
	struct list_head owner_list;
	struct drm_hash_item user_hash;
	void *mm_info;
	struct drm_sman_mm *mm;
	struct drm_sman *sman;
};

struct drm_sman {
	struct drm_sman_mm *mm;
	int num_managers;
	struct drm_open_hash owner_hash_tab;
	struct drm_open_hash user_hash_tab;
	struct list_head owner_items;
};


extern void drm_sman_takedown(struct drm_sman * sman);


extern int drm_sman_init(struct drm_sman * sman, unsigned int num_managers,
			 unsigned int user_order, unsigned int owner_order);


extern int drm_sman_set_range(struct drm_sman * sman, unsigned int manager,
			      unsigned long start, unsigned long size);


extern int drm_sman_set_manager(struct drm_sman * sman, unsigned int mananger,
				struct drm_sman_mm * allocator);


extern struct drm_memblock_item *drm_sman_alloc(struct drm_sman * sman,
						unsigned int manager,
						unsigned long size,
						unsigned alignment,
						unsigned long owner);

extern int drm_sman_free_key(struct drm_sman * sman, unsigned int key);


extern int drm_sman_owner_clean(struct drm_sman * sman, unsigned long owner);


extern void drm_sman_owner_cleanup(struct drm_sman * sman, unsigned long owner);


extern void drm_sman_cleanup(struct drm_sman * sman);

#endif
