

#ifndef __NOUVEAU_FB_H__
#define __NOUVEAU_FB_H__

struct nouveau_framebuffer {
	struct drm_framebuffer base;
	struct nouveau_bo *nvbo;
};

static inline struct nouveau_framebuffer *
nouveau_framebuffer(struct drm_framebuffer *fb)
{
	return container_of(fb, struct nouveau_framebuffer, base);
}

extern const struct drm_mode_config_funcs nouveau_mode_config_funcs;

int nouveau_framebuffer_init(struct drm_device *dev, struct nouveau_framebuffer *nouveau_fb,
			     struct drm_mode_fb_cmd *mode_cmd, struct nouveau_bo *nvbo);
#endif /* __NOUVEAU_FB_H__ */
