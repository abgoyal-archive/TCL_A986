
#ifndef __RADEON_OBJECT_H__
#define __RADEON_OBJECT_H__

#include <drm/radeon_drm.h>
#include "radeon.h"

static inline unsigned radeon_mem_type_to_domain(u32 mem_type)
{
	switch (mem_type) {
	case TTM_PL_VRAM:
		return RADEON_GEM_DOMAIN_VRAM;
	case TTM_PL_TT:
		return RADEON_GEM_DOMAIN_GTT;
	case TTM_PL_SYSTEM:
		return RADEON_GEM_DOMAIN_CPU;
	default:
		break;
	}
	return 0;
}

static inline int radeon_bo_reserve(struct radeon_bo *bo, bool no_wait)
{
	int r;

	r = ttm_bo_reserve(&bo->tbo, true, no_wait, false, 0);
	if (unlikely(r != 0)) {
		if (r != -ERESTARTSYS)
			dev_err(bo->rdev->dev, "%p reserve failed\n", bo);
		return r;
	}
	return 0;
}

static inline void radeon_bo_unreserve(struct radeon_bo *bo)
{
	ttm_bo_unreserve(&bo->tbo);
}

static inline u64 radeon_bo_gpu_offset(struct radeon_bo *bo)
{
	return bo->tbo.offset;
}

static inline unsigned long radeon_bo_size(struct radeon_bo *bo)
{
	return bo->tbo.num_pages << PAGE_SHIFT;
}

static inline bool radeon_bo_is_reserved(struct radeon_bo *bo)
{
	return !!atomic_read(&bo->tbo.reserved);
}

static inline u64 radeon_bo_mmap_offset(struct radeon_bo *bo)
{
	return bo->tbo.addr_space_offset;
}

static inline int radeon_bo_wait(struct radeon_bo *bo, u32 *mem_type,
					bool no_wait)
{
	int r;

	r = ttm_bo_reserve(&bo->tbo, true, no_wait, false, 0);
	if (unlikely(r != 0)) {
		if (r != -ERESTARTSYS)
			dev_err(bo->rdev->dev, "%p reserve failed for wait\n", bo);
		return r;
	}
	spin_lock(&bo->tbo.lock);
	if (mem_type)
		*mem_type = bo->tbo.mem.mem_type;
	if (bo->tbo.sync_obj)
		r = ttm_bo_wait(&bo->tbo, true, true, no_wait);
	spin_unlock(&bo->tbo.lock);
	ttm_bo_unreserve(&bo->tbo);
	return r;
}

extern int radeon_bo_create(struct radeon_device *rdev,
				struct drm_gem_object *gobj, unsigned long size,
				bool kernel, u32 domain,
				struct radeon_bo **bo_ptr);
extern int radeon_bo_kmap(struct radeon_bo *bo, void **ptr);
extern void radeon_bo_kunmap(struct radeon_bo *bo);
extern void radeon_bo_unref(struct radeon_bo **bo);
extern int radeon_bo_pin(struct radeon_bo *bo, u32 domain, u64 *gpu_addr);
extern int radeon_bo_unpin(struct radeon_bo *bo);
extern int radeon_bo_evict_vram(struct radeon_device *rdev);
extern void radeon_bo_force_delete(struct radeon_device *rdev);
extern int radeon_bo_init(struct radeon_device *rdev);
extern void radeon_bo_fini(struct radeon_device *rdev);
extern void radeon_bo_list_add_object(struct radeon_bo_list *lobj,
				struct list_head *head);
extern int radeon_bo_list_reserve(struct list_head *head);
extern void radeon_bo_list_unreserve(struct list_head *head);
extern int radeon_bo_list_validate(struct list_head *head);
extern void radeon_bo_list_fence(struct list_head *head, void *fence);
extern int radeon_bo_fbdev_mmap(struct radeon_bo *bo,
				struct vm_area_struct *vma);
extern int radeon_bo_set_tiling_flags(struct radeon_bo *bo,
				u32 tiling_flags, u32 pitch);
extern void radeon_bo_get_tiling_flags(struct radeon_bo *bo,
				u32 *tiling_flags, u32 *pitch);
extern int radeon_bo_check_tiling(struct radeon_bo *bo, bool has_moved,
				bool force_drop);
extern void radeon_bo_move_notify(struct ttm_buffer_object *bo,
					struct ttm_mem_reg *mem);
extern int radeon_bo_fault_reserve_notify(struct ttm_buffer_object *bo);
extern int radeon_bo_get_surface_reg(struct radeon_bo *bo);
#endif
