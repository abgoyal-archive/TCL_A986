

#ifndef __NOUVEAU_FBCON_H__
#define __NOUVEAU_FBCON_H__

#include "drm_fb_helper.h"

#include "nouveau_fb.h"
struct nouveau_fbdev {
	struct drm_fb_helper helper;
	struct nouveau_framebuffer nouveau_fb;
	struct list_head fbdev_list;
	struct drm_device *dev;
	unsigned int saved_flags;
};

void nouveau_fbcon_restore(void);

void nv04_fbcon_copyarea(struct fb_info *info, const struct fb_copyarea *region);
void nv04_fbcon_fillrect(struct fb_info *info, const struct fb_fillrect *rect);
void nv04_fbcon_imageblit(struct fb_info *info, const struct fb_image *image);
int nv04_fbcon_accel_init(struct fb_info *info);
void nv50_fbcon_fillrect(struct fb_info *info, const struct fb_fillrect *rect);
void nv50_fbcon_copyarea(struct fb_info *info, const struct fb_copyarea *region);
void nv50_fbcon_imageblit(struct fb_info *info, const struct fb_image *image);
int nv50_fbcon_accel_init(struct fb_info *info);

void nouveau_fbcon_gpu_lockup(struct fb_info *info);

int nouveau_fbcon_init(struct drm_device *dev);
void nouveau_fbcon_fini(struct drm_device *dev);
void nouveau_fbcon_set_suspend(struct drm_device *dev, int state);
void nouveau_fbcon_zfill_all(struct drm_device *dev);
void nouveau_fbcon_save_disable_accel(struct drm_device *dev);
void nouveau_fbcon_restore_accel(struct drm_device *dev);

void nouveau_fbcon_output_poll_changed(struct drm_device *dev);
#endif /* __NV50_FBCON_H__ */

