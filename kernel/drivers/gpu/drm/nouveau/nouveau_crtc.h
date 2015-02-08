

#ifndef __NOUVEAU_CRTC_H__
#define __NOUVEAU_CRTC_H__

struct nouveau_crtc {
	struct drm_crtc base;

	int index;

	struct drm_display_mode *mode;

	uint32_t dpms_saved_fp_control;
	uint32_t fp_users;
	int saturation;
	int sharpness;
	int last_dpms;

	int cursor_saved_x, cursor_saved_y;

	struct {
		int cpp;
		bool blanked;
		uint32_t offset;
		uint32_t tile_flags;
	} fb;

	struct {
		struct nouveau_bo *nvbo;
		bool visible;
		uint32_t offset;
		void (*set_offset)(struct nouveau_crtc *, uint32_t offset);
		void (*set_pos)(struct nouveau_crtc *, int x, int y);
		void (*hide)(struct nouveau_crtc *, bool update);
		void (*show)(struct nouveau_crtc *, bool update);
	} cursor;

	struct {
		struct nouveau_bo *nvbo;
		uint16_t r[256];
		uint16_t g[256];
		uint16_t b[256];
		int depth;
	} lut;

	int (*set_dither)(struct nouveau_crtc *crtc, bool on, bool update);
	int (*set_scale)(struct nouveau_crtc *crtc, int mode, bool update);
};

static inline struct nouveau_crtc *nouveau_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct nouveau_crtc, base);
}

static inline struct drm_crtc *to_drm_crtc(struct nouveau_crtc *crtc)
{
	return &crtc->base;
}

int nv50_crtc_create(struct drm_device *dev, int index);
int nv50_cursor_init(struct nouveau_crtc *);
void nv50_cursor_fini(struct nouveau_crtc *);
int nv50_crtc_cursor_set(struct drm_crtc *drm_crtc, struct drm_file *file_priv,
			 uint32_t buffer_handle, uint32_t width,
			 uint32_t height);
int nv50_crtc_cursor_move(struct drm_crtc *drm_crtc, int x, int y);

int nv04_cursor_init(struct nouveau_crtc *);

struct nouveau_connector *
nouveau_crtc_connector_get(struct nouveau_crtc *crtc);

#endif /* __NOUVEAU_CRTC_H__ */
