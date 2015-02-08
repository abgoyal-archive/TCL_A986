

#ifndef __NV50_DISPLAY_H__
#define __NV50_DISPLAY_H__

#include "drmP.h"
#include "drm.h"
#include "nouveau_drv.h"
#include "nouveau_dma.h"
#include "nouveau_reg.h"
#include "nouveau_crtc.h"
#include "nv50_evo.h"

void nv50_display_irq_handler(struct drm_device *dev);
void nv50_display_irq_handler_bh(struct work_struct *work);
void nv50_display_irq_hotplug_bh(struct work_struct *work);
int nv50_display_init(struct drm_device *dev);
int nv50_display_create(struct drm_device *dev);
int nv50_display_destroy(struct drm_device *dev);
int nv50_crtc_blank(struct nouveau_crtc *, bool blank);
int nv50_crtc_set_clock(struct drm_device *, int head, int pclk);

#endif /* __NV50_DISPLAY_H__ */
