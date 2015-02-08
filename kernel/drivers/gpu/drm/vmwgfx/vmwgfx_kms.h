

#ifndef VMWGFX_KMS_H_
#define VMWGFX_KMS_H_

#include "drmP.h"
#include "vmwgfx_drv.h"


#define vmw_framebuffer_to_vfb(x) \
	container_of(x, struct vmw_framebuffer, base)

struct vmw_framebuffer {
	struct drm_framebuffer base;
	int (*pin)(struct vmw_framebuffer *fb);
	int (*unpin)(struct vmw_framebuffer *fb);
};


#define vmw_crtc_to_du(x) \
	container_of(x, struct vmw_display_unit, crtc)

int vmw_cursor_update_image(struct vmw_private *dev_priv,
			    u32 *image, u32 width, u32 height,
			    u32 hotspotX, u32 hotspotY);
void vmw_cursor_update_position(struct vmw_private *dev_priv,
				bool show, int x, int y);

struct vmw_display_unit {
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;

	struct vmw_surface *cursor_surface;
	struct vmw_dma_buffer *cursor_dmabuf;
	size_t cursor_age;

	int cursor_x;
	int cursor_y;

	int hotspot_x;
	int hotspot_y;

	unsigned unit;
};

void vmw_display_unit_cleanup(struct vmw_display_unit *du);
int vmw_du_crtc_cursor_set(struct drm_crtc *crtc, struct drm_file *file_priv,
			   uint32_t handle, uint32_t width, uint32_t height);
int vmw_du_crtc_cursor_move(struct drm_crtc *crtc, int x, int y);

int vmw_kms_init_legacy_display_system(struct vmw_private *dev_priv);
int vmw_kms_close_legacy_display_system(struct vmw_private *dev_priv);
int vmw_kms_ldu_update_layout(struct vmw_private *dev_priv, unsigned num,
			      struct drm_vmw_rect *rects);

#endif
