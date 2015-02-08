

#ifndef __VMWGFX_DRM_H__
#define __VMWGFX_DRM_H__

#define DRM_VMW_MAX_SURFACE_FACES 6
#define DRM_VMW_MAX_MIP_LEVELS 24

#define DRM_VMW_EXT_NAME_LEN 128

#define DRM_VMW_GET_PARAM            0
#define DRM_VMW_ALLOC_DMABUF         1
#define DRM_VMW_UNREF_DMABUF         2
#define DRM_VMW_CURSOR_BYPASS        3
/* guarded by DRM_VMW_PARAM_NUM_STREAMS != 0*/
#define DRM_VMW_CONTROL_STREAM       4
#define DRM_VMW_CLAIM_STREAM         5
#define DRM_VMW_UNREF_STREAM         6
/* guarded by DRM_VMW_PARAM_3D == 1 */
#define DRM_VMW_CREATE_CONTEXT       7
#define DRM_VMW_UNREF_CONTEXT        8
#define DRM_VMW_CREATE_SURFACE       9
#define DRM_VMW_UNREF_SURFACE        10
#define DRM_VMW_REF_SURFACE          11
#define DRM_VMW_EXECBUF              12
#define DRM_VMW_FIFO_DEBUG           13
#define DRM_VMW_FENCE_WAIT           14
/* guarded by minor version >= 2 */
#define DRM_VMW_UPDATE_LAYOUT        15


/*************************************************************************/

#define DRM_VMW_PARAM_NUM_STREAMS      0
#define DRM_VMW_PARAM_NUM_FREE_STREAMS 1
#define DRM_VMW_PARAM_3D               2
#define DRM_VMW_PARAM_FIFO_OFFSET      3
#define DRM_VMW_PARAM_HW_CAPS          4
#define DRM_VMW_PARAM_FIFO_CAPS        5


struct drm_vmw_getparam_arg {
	uint64_t value;
	uint32_t param;
	uint32_t pad64;
};

/*************************************************************************/


struct drm_vmw_extension_rep {
	int32_t exists;
	uint32_t driver_ioctl_offset;
	uint32_t driver_sarea_offset;
	uint32_t major;
	uint32_t minor;
	uint32_t pl;
	uint32_t pad64;
};


union drm_vmw_extension_arg {
	char extension[DRM_VMW_EXT_NAME_LEN];
	struct drm_vmw_extension_rep rep;
};

/*************************************************************************/


struct drm_vmw_context_arg {
	int32_t cid;
	uint32_t pad64;
};

/*************************************************************************/

/*************************************************************************/


struct drm_vmw_surface_create_req {
	uint32_t flags;
	uint32_t format;
	uint32_t mip_levels[DRM_VMW_MAX_SURFACE_FACES];
	uint64_t size_addr;
	int32_t shareable;
	int32_t scanout;
};


struct drm_vmw_surface_arg {
	int32_t sid;
	uint32_t pad64;
};


struct drm_vmw_size {
	uint32_t width;
	uint32_t height;
	uint32_t depth;
	uint32_t pad64;
};


union drm_vmw_surface_create_arg {
	struct drm_vmw_surface_arg rep;
	struct drm_vmw_surface_create_req req;
};

/*************************************************************************/


union drm_vmw_surface_reference_arg {
	struct drm_vmw_surface_create_req rep;
	struct drm_vmw_surface_arg req;
};

/*************************************************************************/

/*************************************************************************/


#define DRM_VMW_EXECBUF_VERSION 0

struct drm_vmw_execbuf_arg {
	uint64_t commands;
	uint32_t command_size;
	uint32_t throttle_us;
	uint64_t fence_rep;
	 uint32_t version;
	 uint32_t flags;
};


struct drm_vmw_fence_rep {
	uint64_t fence_seq;
	int32_t error;
	uint32_t pad64;
};

/*************************************************************************/


struct drm_vmw_alloc_dmabuf_req {
	uint32_t size;
	uint32_t pad64;
};


struct drm_vmw_dmabuf_rep {
	uint64_t map_handle;
	uint32_t handle;
	uint32_t cur_gmr_id;
	uint32_t cur_gmr_offset;
	uint32_t pad64;
};


union drm_vmw_alloc_dmabuf_arg {
	struct drm_vmw_alloc_dmabuf_req req;
	struct drm_vmw_dmabuf_rep rep;
};

/*************************************************************************/


struct drm_vmw_unref_dmabuf_arg {
	uint32_t handle;
	uint32_t pad64;
};

/*************************************************************************/


struct drm_vmw_fifo_debug_arg {
	uint64_t debug_buffer;
	uint32_t debug_buffer_size;
	uint32_t used_size;
	int32_t did_not_fit;
	uint32_t pad64;
};

struct drm_vmw_fence_wait_arg {
	uint64_t sequence;
	uint64_t kernel_cookie;
	int32_t cookie_valid;
	int32_t pad64;
};

/*************************************************************************/


struct drm_vmw_rect {
	int32_t x;
	int32_t y;
	uint32_t w;
	uint32_t h;
};


struct drm_vmw_control_stream_arg {
	uint32_t stream_id;
	uint32_t enabled;

	uint32_t flags;
	uint32_t color_key;

	uint32_t handle;
	uint32_t offset;
	int32_t format;
	uint32_t size;
	uint32_t width;
	uint32_t height;
	uint32_t pitch[3];

	uint32_t pad64;
	struct drm_vmw_rect src;
	struct drm_vmw_rect dst;
};

/*************************************************************************/

#define DRM_VMW_CURSOR_BYPASS_ALL    (1 << 0)
#define DRM_VMW_CURSOR_BYPASS_FLAGS       (1)


struct drm_vmw_cursor_bypass_arg {
	uint32_t flags;
	uint32_t crtc_id;
	int32_t xpos;
	int32_t ypos;
	int32_t xhot;
	int32_t yhot;
};

/*************************************************************************/


struct drm_vmw_stream_arg {
	uint32_t stream_id;
	uint32_t pad64;
};

/*************************************************************************/

/*************************************************************************/


struct drm_vmw_update_layout_arg {
	uint32_t num_outputs;
	uint32_t pad64;
	uint64_t rects;
};

#endif
