

#include "linux/string.h"
#include "linux/bitops.h"
#include "drmP.h"
#include "drm.h"
#include "i915_drm.h"
#include "i915_drv.h"


void
i915_gem_detect_bit_6_swizzle(struct drm_device *dev)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	uint32_t swizzle_x = I915_BIT_6_SWIZZLE_UNKNOWN;
	uint32_t swizzle_y = I915_BIT_6_SWIZZLE_UNKNOWN;

	if (IS_IRONLAKE(dev) || IS_GEN6(dev)) {
		/* On Ironlake whatever DRAM config, GPU always do
		 * same swizzling setup.
		 */
		swizzle_x = I915_BIT_6_SWIZZLE_9_10;
		swizzle_y = I915_BIT_6_SWIZZLE_9;
	} else if (!IS_I9XX(dev)) {
		/* As far as we know, the 865 doesn't have these bit 6
		 * swizzling issues.
		 */
		swizzle_x = I915_BIT_6_SWIZZLE_NONE;
		swizzle_y = I915_BIT_6_SWIZZLE_NONE;
	} else if (IS_MOBILE(dev)) {
		uint32_t dcc;

		/* On mobile 9xx chipsets, channel interleave by the CPU is
		 * determined by DCC.  For single-channel, neither the CPU
		 * nor the GPU do swizzling.  For dual channel interleaved,
		 * the GPU's interleave is bit 9 and 10 for X tiled, and bit
		 * 9 for Y tiled.  The CPU's interleave is independent, and
		 * can be based on either bit 11 (haven't seen this yet) or
		 * bit 17 (common).
		 */
		dcc = I915_READ(DCC);
		switch (dcc & DCC_ADDRESSING_MODE_MASK) {
		case DCC_ADDRESSING_MODE_SINGLE_CHANNEL:
		case DCC_ADDRESSING_MODE_DUAL_CHANNEL_ASYMMETRIC:
			swizzle_x = I915_BIT_6_SWIZZLE_NONE;
			swizzle_y = I915_BIT_6_SWIZZLE_NONE;
			break;
		case DCC_ADDRESSING_MODE_DUAL_CHANNEL_INTERLEAVED:
			if (dcc & DCC_CHANNEL_XOR_DISABLE) {
				/* This is the base swizzling by the GPU for
				 * tiled buffers.
				 */
				swizzle_x = I915_BIT_6_SWIZZLE_9_10;
				swizzle_y = I915_BIT_6_SWIZZLE_9;
			} else if ((dcc & DCC_CHANNEL_XOR_BIT_17) == 0) {
				/* Bit 11 swizzling by the CPU in addition. */
				swizzle_x = I915_BIT_6_SWIZZLE_9_10_11;
				swizzle_y = I915_BIT_6_SWIZZLE_9_11;
			} else {
				/* Bit 17 swizzling by the CPU in addition. */
				swizzle_x = I915_BIT_6_SWIZZLE_9_10_17;
				swizzle_y = I915_BIT_6_SWIZZLE_9_17;
			}
			break;
		}
		if (dcc == 0xffffffff) {
			DRM_ERROR("Couldn't read from MCHBAR.  "
				  "Disabling tiling.\n");
			swizzle_x = I915_BIT_6_SWIZZLE_UNKNOWN;
			swizzle_y = I915_BIT_6_SWIZZLE_UNKNOWN;
		}
	} else {
		/* The 965, G33, and newer, have a very flexible memory
		 * configuration.  It will enable dual-channel mode
		 * (interleaving) on as much memory as it can, and the GPU
		 * will additionally sometimes enable different bit 6
		 * swizzling for tiled objects from the CPU.
		 *
		 * Here's what I found on the G965:
		 *    slot fill         memory size  swizzling
		 * 0A   0B   1A   1B    1-ch   2-ch
		 * 512  0    0    0     512    0     O
		 * 512  0    512  0     16     1008  X
		 * 512  0    0    512   16     1008  X
		 * 0    512  0    512   16     1008  X
		 * 1024 1024 1024 0     2048   1024  O
		 *
		 * We could probably detect this based on either the DRB
		 * matching, which was the case for the swizzling required in
		 * the table above, or from the 1-ch value being less than
		 * the minimum size of a rank.
		 */
		if (I915_READ16(C0DRB3) != I915_READ16(C1DRB3)) {
			swizzle_x = I915_BIT_6_SWIZZLE_NONE;
			swizzle_y = I915_BIT_6_SWIZZLE_NONE;
		} else {
			swizzle_x = I915_BIT_6_SWIZZLE_9_10;
			swizzle_y = I915_BIT_6_SWIZZLE_9;
		}
	}

	dev_priv->mm.bit_6_swizzle_x = swizzle_x;
	dev_priv->mm.bit_6_swizzle_y = swizzle_y;
}

/* Check pitch constriants for all chips & tiling formats */
bool
i915_tiling_ok(struct drm_device *dev, int stride, int size, int tiling_mode)
{
	int tile_width;

	/* Linear is always fine */
	if (tiling_mode == I915_TILING_NONE)
		return true;

	if (!IS_I9XX(dev) ||
	    (tiling_mode == I915_TILING_Y && HAS_128_BYTE_Y_TILING(dev)))
		tile_width = 128;
	else
		tile_width = 512;

	/* check maximum stride & object size */
	if (IS_I965G(dev)) {
		/* i965 stores the end address of the gtt mapping in the fence
		 * reg, so dont bother to check the size */
		if (stride / 128 > I965_FENCE_MAX_PITCH_VAL)
			return false;
	} else if (IS_GEN3(dev) || IS_GEN2(dev)) {
		if (stride > 8192)
			return false;

		if (IS_GEN3(dev)) {
			if (size > I830_FENCE_MAX_SIZE_VAL << 20)
				return false;
		} else {
			if (size > I830_FENCE_MAX_SIZE_VAL << 19)
				return false;
		}
	}

	/* 965+ just needs multiples of tile width */
	if (IS_I965G(dev)) {
		if (stride & (tile_width - 1))
			return false;
		return true;
	}

	/* Pre-965 needs power of two tile widths */
	if (stride < tile_width)
		return false;

	if (stride & (stride - 1))
		return false;

	return true;
}

bool
i915_gem_object_fence_offset_ok(struct drm_gem_object *obj, int tiling_mode)
{
	struct drm_device *dev = obj->dev;
	struct drm_i915_gem_object *obj_priv = to_intel_bo(obj);

	if (obj_priv->gtt_space == NULL)
		return true;

	if (tiling_mode == I915_TILING_NONE)
		return true;

	if (!IS_I965G(dev)) {
		if (obj_priv->gtt_offset & (obj->size - 1))
			return false;
		if (IS_I9XX(dev)) {
			if (obj_priv->gtt_offset & ~I915_FENCE_START_MASK)
				return false;
		} else {
			if (obj_priv->gtt_offset & ~I830_FENCE_START_MASK)
				return false;
		}
	}

	return true;
}

int
i915_gem_set_tiling(struct drm_device *dev, void *data,
		   struct drm_file *file_priv)
{
	struct drm_i915_gem_set_tiling *args = data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_gem_object *obj;
	struct drm_i915_gem_object *obj_priv;
	int ret = 0;

	obj = drm_gem_object_lookup(dev, file_priv, args->handle);
	if (obj == NULL)
		return -EINVAL;
	obj_priv = to_intel_bo(obj);

	if (!i915_tiling_ok(dev, args->stride, obj->size, args->tiling_mode)) {
		drm_gem_object_unreference_unlocked(obj);
		return -EINVAL;
	}

	if (obj_priv->pin_count) {
		drm_gem_object_unreference_unlocked(obj);
		return -EBUSY;
	}

	if (args->tiling_mode == I915_TILING_NONE) {
		args->swizzle_mode = I915_BIT_6_SWIZZLE_NONE;
		args->stride = 0;
	} else {
		if (args->tiling_mode == I915_TILING_X)
			args->swizzle_mode = dev_priv->mm.bit_6_swizzle_x;
		else
			args->swizzle_mode = dev_priv->mm.bit_6_swizzle_y;

		/* Hide bit 17 swizzling from the user.  This prevents old Mesa
		 * from aborting the application on sw fallbacks to bit 17,
		 * and we use the pread/pwrite bit17 paths to swizzle for it.
		 * If there was a user that was relying on the swizzle
		 * information for drm_intel_bo_map()ed reads/writes this would
		 * break it, but we don't have any of those.
		 */
		if (args->swizzle_mode == I915_BIT_6_SWIZZLE_9_17)
			args->swizzle_mode = I915_BIT_6_SWIZZLE_9;
		if (args->swizzle_mode == I915_BIT_6_SWIZZLE_9_10_17)
			args->swizzle_mode = I915_BIT_6_SWIZZLE_9_10;

		/* If we can't handle the swizzling, make it untiled. */
		if (args->swizzle_mode == I915_BIT_6_SWIZZLE_UNKNOWN) {
			args->tiling_mode = I915_TILING_NONE;
			args->swizzle_mode = I915_BIT_6_SWIZZLE_NONE;
			args->stride = 0;
		}
	}

	mutex_lock(&dev->struct_mutex);
	if (args->tiling_mode != obj_priv->tiling_mode ||
	    args->stride != obj_priv->stride) {
		/* We need to rebind the object if its current allocation
		 * no longer meets the alignment restrictions for its new
		 * tiling mode. Otherwise we can just leave it alone, but
		 * need to ensure that any fence register is cleared.
		 */
		if (!i915_gem_object_fence_offset_ok(obj, args->tiling_mode))
			ret = i915_gem_object_unbind(obj);
		else if (obj_priv->fence_reg != I915_FENCE_REG_NONE)
			ret = i915_gem_object_put_fence_reg(obj);
		else
			i915_gem_release_mmap(obj);

		if (ret != 0) {
			WARN(ret != -ERESTARTSYS,
			     "failed to reset object for tiling switch");
			args->tiling_mode = obj_priv->tiling_mode;
			args->stride = obj_priv->stride;
			goto err;
		}

		obj_priv->tiling_mode = args->tiling_mode;
		obj_priv->stride = args->stride;
	}
err:
	drm_gem_object_unreference(obj);
	mutex_unlock(&dev->struct_mutex);

	return ret;
}

int
i915_gem_get_tiling(struct drm_device *dev, void *data,
		   struct drm_file *file_priv)
{
	struct drm_i915_gem_get_tiling *args = data;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_gem_object *obj;
	struct drm_i915_gem_object *obj_priv;

	obj = drm_gem_object_lookup(dev, file_priv, args->handle);
	if (obj == NULL)
		return -EINVAL;
	obj_priv = to_intel_bo(obj);

	mutex_lock(&dev->struct_mutex);

	args->tiling_mode = obj_priv->tiling_mode;
	switch (obj_priv->tiling_mode) {
	case I915_TILING_X:
		args->swizzle_mode = dev_priv->mm.bit_6_swizzle_x;
		break;
	case I915_TILING_Y:
		args->swizzle_mode = dev_priv->mm.bit_6_swizzle_y;
		break;
	case I915_TILING_NONE:
		args->swizzle_mode = I915_BIT_6_SWIZZLE_NONE;
		break;
	default:
		DRM_ERROR("unknown tiling mode\n");
	}

	/* Hide bit 17 from the user -- see comment in i915_gem_set_tiling */
	if (args->swizzle_mode == I915_BIT_6_SWIZZLE_9_17)
		args->swizzle_mode = I915_BIT_6_SWIZZLE_9;
	if (args->swizzle_mode == I915_BIT_6_SWIZZLE_9_10_17)
		args->swizzle_mode = I915_BIT_6_SWIZZLE_9_10;

	drm_gem_object_unreference(obj);
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

static int
i915_gem_swizzle_page(struct page *page)
{
	char *vaddr;
	int i;
	char temp[64];

	vaddr = kmap(page);
	if (vaddr == NULL)
		return -ENOMEM;

	for (i = 0; i < PAGE_SIZE; i += 128) {
		memcpy(temp, &vaddr[i], 64);
		memcpy(&vaddr[i], &vaddr[i + 64], 64);
		memcpy(&vaddr[i + 64], temp, 64);
	}

	kunmap(page);

	return 0;
}

void
i915_gem_object_do_bit_17_swizzle(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj_priv = to_intel_bo(obj);
	int page_count = obj->size >> PAGE_SHIFT;
	int i;

	if (dev_priv->mm.bit_6_swizzle_x != I915_BIT_6_SWIZZLE_9_10_17)
		return;

	if (obj_priv->bit_17 == NULL)
		return;

	for (i = 0; i < page_count; i++) {
		char new_bit_17 = page_to_phys(obj_priv->pages[i]) >> 17;
		if ((new_bit_17 & 0x1) !=
		    (test_bit(i, obj_priv->bit_17) != 0)) {
			int ret = i915_gem_swizzle_page(obj_priv->pages[i]);
			if (ret != 0) {
				DRM_ERROR("Failed to swizzle page\n");
				return;
			}
			set_page_dirty(obj_priv->pages[i]);
		}
	}
}

void
i915_gem_object_save_bit_17_swizzle(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_i915_gem_object *obj_priv = to_intel_bo(obj);
	int page_count = obj->size >> PAGE_SHIFT;
	int i;

	if (dev_priv->mm.bit_6_swizzle_x != I915_BIT_6_SWIZZLE_9_10_17)
		return;

	if (obj_priv->bit_17 == NULL) {
		obj_priv->bit_17 = kmalloc(BITS_TO_LONGS(page_count) *
					   sizeof(long), GFP_KERNEL);
		if (obj_priv->bit_17 == NULL) {
			DRM_ERROR("Failed to allocate memory for bit 17 "
				  "record\n");
			return;
		}
	}

	for (i = 0; i < page_count; i++) {
		if (page_to_phys(obj_priv->pages[i]) & (1 << 17))
			__set_bit(i, obj_priv->bit_17);
		else
			__clear_bit(i, obj_priv->bit_17);
	}
}
