
#ifndef _TTM_BO_DRIVER_H_
#define _TTM_BO_DRIVER_H_

#include "ttm/ttm_bo_api.h"
#include "ttm/ttm_memory.h"
#include "ttm/ttm_module.h"
#include "drm_mm.h"
#include "linux/workqueue.h"
#include "linux/fs.h"
#include "linux/spinlock.h"

struct ttm_backend;

struct ttm_backend_func {
	/**
	 * struct ttm_backend_func member populate
	 *
	 * @backend: Pointer to a struct ttm_backend.
	 * @num_pages: Number of pages to populate.
	 * @pages: Array of pointers to ttm pages.
	 * @dummy_read_page: Page to be used instead of NULL pages in the
	 * array @pages.
	 *
	 * Populate the backend with ttm pages. Depending on the backend,
	 * it may or may not copy the @pages array.
	 */
	int (*populate) (struct ttm_backend *backend,
			 unsigned long num_pages, struct page **pages,
			 struct page *dummy_read_page);
	/**
	 * struct ttm_backend_func member clear
	 *
	 * @backend: Pointer to a struct ttm_backend.
	 *
	 * This is an "unpopulate" function. Release all resources
	 * allocated with populate.
	 */
	void (*clear) (struct ttm_backend *backend);

	/**
	 * struct ttm_backend_func member bind
	 *
	 * @backend: Pointer to a struct ttm_backend.
	 * @bo_mem: Pointer to a struct ttm_mem_reg describing the
	 * memory type and location for binding.
	 *
	 * Bind the backend pages into the aperture in the location
	 * indicated by @bo_mem. This function should be able to handle
	 * differences between aperture- and system page sizes.
	 */
	int (*bind) (struct ttm_backend *backend, struct ttm_mem_reg *bo_mem);

	/**
	 * struct ttm_backend_func member unbind
	 *
	 * @backend: Pointer to a struct ttm_backend.
	 *
	 * Unbind previously bound backend pages. This function should be
	 * able to handle differences between aperture- and system page sizes.
	 */
	int (*unbind) (struct ttm_backend *backend);

	/**
	 * struct ttm_backend_func member destroy
	 *
	 * @backend: Pointer to a struct ttm_backend.
	 *
	 * Destroy the backend.
	 */
	void (*destroy) (struct ttm_backend *backend);
};


struct ttm_backend {
	struct ttm_bo_device *bdev;
	uint32_t flags;
	struct ttm_backend_func *func;
};

#define TTM_PAGE_FLAG_USER            (1 << 1)
#define TTM_PAGE_FLAG_USER_DIRTY      (1 << 2)
#define TTM_PAGE_FLAG_WRITE           (1 << 3)
#define TTM_PAGE_FLAG_SWAPPED         (1 << 4)
#define TTM_PAGE_FLAG_PERSISTANT_SWAP (1 << 5)
#define TTM_PAGE_FLAG_ZERO_ALLOC      (1 << 6)
#define TTM_PAGE_FLAG_DMA32           (1 << 7)

enum ttm_caching_state {
	tt_uncached,
	tt_wc,
	tt_cached
};


struct ttm_tt {
	struct page *dummy_read_page;
	struct page **pages;
	long first_himem_page;
	long last_lomem_page;
	uint32_t page_flags;
	unsigned long num_pages;
	struct ttm_bo_global *glob;
	struct ttm_backend *be;
	struct task_struct *tsk;
	unsigned long start;
	struct file *swap_storage;
	enum ttm_caching_state caching_state;
	enum {
		tt_bound,
		tt_unbound,
		tt_unpopulated,
	} state;
};

#define TTM_MEMTYPE_FLAG_FIXED         (1 << 0)	/* Fixed (on-card) PCI memory */
#define TTM_MEMTYPE_FLAG_MAPPABLE      (1 << 1)	/* Memory mappable */
#define TTM_MEMTYPE_FLAG_CMA           (1 << 3)	/* Can't map aperture */


struct ttm_mem_type_manager {

	/*
	 * No protection. Constant from start.
	 */

	bool has_type;
	bool use_type;
	uint32_t flags;
	unsigned long gpu_offset;
	uint64_t size;
	uint32_t available_caching;
	uint32_t default_caching;

	/*
	 * Protected by the bdev->lru_lock.
	 * TODO: Consider one lru_lock per ttm_mem_type_manager.
	 * Plays ill with list removal, though.
	 */

	struct drm_mm manager;
	struct list_head lru;
};


struct ttm_bo_driver {
	/**
	 * struct ttm_bo_driver member create_ttm_backend_entry
	 *
	 * @bdev: The buffer object device.
	 *
	 * Create a driver specific struct ttm_backend.
	 */

	struct ttm_backend *(*create_ttm_backend_entry)
	 (struct ttm_bo_device *bdev);

	/**
	 * struct ttm_bo_driver member invalidate_caches
	 *
	 * @bdev: the buffer object device.
	 * @flags: new placement of the rebound buffer object.
	 *
	 * A previosly evicted buffer has been rebound in a
	 * potentially new location. Tell the driver that it might
	 * consider invalidating read (texture) caches on the next command
	 * submission as a consequence.
	 */

	int (*invalidate_caches) (struct ttm_bo_device *bdev, uint32_t flags);
	int (*init_mem_type) (struct ttm_bo_device *bdev, uint32_t type,
			      struct ttm_mem_type_manager *man);
	/**
	 * struct ttm_bo_driver member evict_flags:
	 *
	 * @bo: the buffer object to be evicted
	 *
	 * Return the bo flags for a buffer which is not mapped to the hardware.
	 * These will be placed in proposed_flags so that when the move is
	 * finished, they'll end up in bo->mem.flags
	 */

	 void(*evict_flags) (struct ttm_buffer_object *bo,
				struct ttm_placement *placement);
	/**
	 * struct ttm_bo_driver member move:
	 *
	 * @bo: the buffer to move
	 * @evict: whether this motion is evicting the buffer from
	 * the graphics address space
	 * @interruptible: Use interruptible sleeps if possible when sleeping.
	 * @no_wait: whether this should give up and return -EBUSY
	 * if this move would require sleeping
	 * @new_mem: the new memory region receiving the buffer
	 *
	 * Move a buffer between two memory regions.
	 */
	int (*move) (struct ttm_buffer_object *bo,
		     bool evict, bool interruptible,
		     bool no_wait_reserve, bool no_wait_gpu,
		     struct ttm_mem_reg *new_mem);

	/**
	 * struct ttm_bo_driver_member verify_access
	 *
	 * @bo: Pointer to a buffer object.
	 * @filp: Pointer to a struct file trying to access the object.
	 *
	 * Called from the map / write / read methods to verify that the
	 * caller is permitted to access the buffer object.
	 * This member may be set to NULL, which will refuse this kind of
	 * access for all buffer objects.
	 * This function should return 0 if access is granted, -EPERM otherwise.
	 */
	int (*verify_access) (struct ttm_buffer_object *bo,
			      struct file *filp);

	/**
	 * In case a driver writer dislikes the TTM fence objects,
	 * the driver writer can replace those with sync objects of
	 * his / her own. If it turns out that no driver writer is
	 * using these. I suggest we remove these hooks and plug in
	 * fences directly. The bo driver needs the following functionality:
	 * See the corresponding functions in the fence object API
	 * documentation.
	 */

	bool (*sync_obj_signaled) (void *sync_obj, void *sync_arg);
	int (*sync_obj_wait) (void *sync_obj, void *sync_arg,
			      bool lazy, bool interruptible);
	int (*sync_obj_flush) (void *sync_obj, void *sync_arg);
	void (*sync_obj_unref) (void **sync_obj);
	void *(*sync_obj_ref) (void *sync_obj);

	/* hook to notify driver about a driver move so it
	 * can do tiling things */
	void (*move_notify)(struct ttm_buffer_object *bo,
			    struct ttm_mem_reg *new_mem);
	/* notify the driver we are taking a fault on this BO
	 * and have reserved it */
	int (*fault_reserve_notify)(struct ttm_buffer_object *bo);

	/**
	 * notify the driver that we're about to swap out this bo
	 */
	void (*swap_notify) (struct ttm_buffer_object *bo);

	/**
	 * Driver callback on when mapping io memory (for bo_move_memcpy
	 * for instance). TTM will take care to call io_mem_free whenever
	 * the mapping is not use anymore. io_mem_reserve & io_mem_free
	 * are balanced.
	 */
	int (*io_mem_reserve)(struct ttm_bo_device *bdev, struct ttm_mem_reg *mem);
	void (*io_mem_free)(struct ttm_bo_device *bdev, struct ttm_mem_reg *mem);
};


struct ttm_bo_global_ref {
	struct ttm_global_reference ref;
	struct ttm_mem_global *mem_glob;
};


struct ttm_bo_global {

	/**
	 * Constant after init.
	 */

	struct kobject kobj;
	struct ttm_mem_global *mem_glob;
	struct page *dummy_read_page;
	struct ttm_mem_shrink shrink;
	size_t ttm_bo_extra_size;
	size_t ttm_bo_size;
	struct mutex device_list_mutex;
	spinlock_t lru_lock;

	/**
	 * Protected by device_list_mutex.
	 */
	struct list_head device_list;

	/**
	 * Protected by the lru_lock.
	 */
	struct list_head swap_lru;

	/**
	 * Internal protection.
	 */
	atomic_t bo_count;
};


#define TTM_NUM_MEM_TYPES 8

#define TTM_BO_PRIV_FLAG_MOVING  0	/* Buffer object is moving and needs
					   idling before CPU mapping */
#define TTM_BO_PRIV_FLAG_MAX 1

struct ttm_bo_device {

	/*
	 * Constant after bo device init / atomic.
	 */
	struct list_head device_list;
	struct ttm_bo_global *glob;
	struct ttm_bo_driver *driver;
	rwlock_t vm_lock;
	struct ttm_mem_type_manager man[TTM_NUM_MEM_TYPES];
	/*
	 * Protected by the vm lock.
	 */
	struct rb_root addr_space_rb;
	struct drm_mm addr_space_mm;

	/*
	 * Protected by the global:lru lock.
	 */
	struct list_head ddestroy;

	/*
	 * Protected by load / firstopen / lastclose /unload sync.
	 */

	bool nice_mode;
	struct address_space *dev_mapping;

	/*
	 * Internal protection.
	 */

	struct delayed_work wq;

	bool need_dma32;
};


static inline uint32_t
ttm_flag_masked(uint32_t *old, uint32_t new, uint32_t mask)
{
	*old ^= (*old ^ new) & mask;
	return *old;
}

extern struct ttm_tt *ttm_tt_create(struct ttm_bo_device *bdev,
				    unsigned long size,
				    uint32_t page_flags,
				    struct page *dummy_read_page);


extern int ttm_tt_set_user(struct ttm_tt *ttm,
			   struct task_struct *tsk,
			   unsigned long start, unsigned long num_pages);

extern int ttm_tt_bind(struct ttm_tt *ttm, struct ttm_mem_reg *bo_mem);

extern int ttm_tt_populate(struct ttm_tt *ttm);

extern void ttm_tt_destroy(struct ttm_tt *ttm);

extern void ttm_tt_unbind(struct ttm_tt *ttm);

extern struct page *ttm_tt_get_page(struct ttm_tt *ttm, int index);

extern void ttm_tt_cache_flush(struct page *pages[], unsigned long num_pages);

extern int ttm_tt_set_placement_caching(struct ttm_tt *ttm, uint32_t placement);
extern int ttm_tt_swapout(struct ttm_tt *ttm,
			  struct file *persistant_swap_storage);


extern bool ttm_mem_reg_is_pci(struct ttm_bo_device *bdev,
				   struct ttm_mem_reg *mem);

extern int ttm_bo_mem_space(struct ttm_buffer_object *bo,
				struct ttm_placement *placement,
				struct ttm_mem_reg *mem,
				bool interruptible,
				bool no_wait_reserve, bool no_wait_gpu);

extern int ttm_bo_wait_cpu(struct ttm_buffer_object *bo, bool no_wait);


extern int ttm_bo_pci_offset(struct ttm_bo_device *bdev,
			     struct ttm_mem_reg *mem,
			     unsigned long *bus_base,
			     unsigned long *bus_offset,
			     unsigned long *bus_size);

extern int ttm_mem_io_reserve(struct ttm_bo_device *bdev,
				struct ttm_mem_reg *mem);
extern void ttm_mem_io_free(struct ttm_bo_device *bdev,
				struct ttm_mem_reg *mem);

extern void ttm_bo_global_release(struct ttm_global_reference *ref);
extern int ttm_bo_global_init(struct ttm_global_reference *ref);

extern int ttm_bo_device_release(struct ttm_bo_device *bdev);

extern int ttm_bo_device_init(struct ttm_bo_device *bdev,
			      struct ttm_bo_global *glob,
			      struct ttm_bo_driver *driver,
			      uint64_t file_page_offset, bool need_dma32);

extern void ttm_bo_unmap_virtual(struct ttm_buffer_object *bo);

extern int ttm_bo_reserve(struct ttm_buffer_object *bo,
			  bool interruptible,
			  bool no_wait, bool use_sequence, uint32_t sequence);

extern void ttm_bo_unreserve(struct ttm_buffer_object *bo);

extern int ttm_bo_wait_unreserved(struct ttm_buffer_object *bo,
				  bool interruptible);



extern int ttm_bo_move_ttm(struct ttm_buffer_object *bo,
			   bool evict, bool no_wait_reserve,
			   bool no_wait_gpu, struct ttm_mem_reg *new_mem);


extern int ttm_bo_move_memcpy(struct ttm_buffer_object *bo,
			      bool evict, bool no_wait_reserve,
			      bool no_wait_gpu, struct ttm_mem_reg *new_mem);

extern void ttm_bo_free_old_node(struct ttm_buffer_object *bo);


extern int ttm_bo_move_accel_cleanup(struct ttm_buffer_object *bo,
				     void *sync_obj,
				     void *sync_obj_arg,
				     bool evict, bool no_wait_reserve,
				     bool no_wait_gpu,
				     struct ttm_mem_reg *new_mem);
extern pgprot_t ttm_io_prot(uint32_t caching_flags, pgprot_t tmp);

#if (defined(CONFIG_AGP) || (defined(CONFIG_AGP_MODULE) && defined(MODULE)))
#define TTM_HAS_AGP
#include <linux/agp_backend.h>

extern struct ttm_backend *ttm_agp_backend_init(struct ttm_bo_device *bdev,
						struct agp_bridge_data *bridge);
#endif

#endif
