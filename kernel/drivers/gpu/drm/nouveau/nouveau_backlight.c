


#include <linux/backlight.h>

#include "drmP.h"
#include "nouveau_drv.h"
#include "nouveau_drm.h"
#include "nouveau_reg.h"

static int nv40_get_intensity(struct backlight_device *bd)
{
	struct drm_device *dev = bl_get_data(bd);
	int val = (nv_rd32(dev, NV40_PMC_BACKLIGHT) & NV40_PMC_BACKLIGHT_MASK)
									>> 16;

	return val;
}

static int nv40_set_intensity(struct backlight_device *bd)
{
	struct drm_device *dev = bl_get_data(bd);
	int val = bd->props.brightness;
	int reg = nv_rd32(dev, NV40_PMC_BACKLIGHT);

	nv_wr32(dev, NV40_PMC_BACKLIGHT,
		 (val << 16) | (reg & ~NV40_PMC_BACKLIGHT_MASK));

	return 0;
}

static struct backlight_ops nv40_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = nv40_get_intensity,
	.update_status = nv40_set_intensity,
};

static int nv50_get_intensity(struct backlight_device *bd)
{
	struct drm_device *dev = bl_get_data(bd);

	return nv_rd32(dev, NV50_PDISPLAY_SOR_BACKLIGHT);
}

static int nv50_set_intensity(struct backlight_device *bd)
{
	struct drm_device *dev = bl_get_data(bd);
	int val = bd->props.brightness;

	nv_wr32(dev, NV50_PDISPLAY_SOR_BACKLIGHT,
		val | NV50_PDISPLAY_SOR_BACKLIGHT_ENABLE);
	return 0;
}

static struct backlight_ops nv50_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = nv50_get_intensity,
	.update_status = nv50_set_intensity,
};

static int nouveau_nv40_backlight_init(struct drm_device *dev)
{
	struct backlight_properties props;
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	struct backlight_device *bd;

	if (!(nv_rd32(dev, NV40_PMC_BACKLIGHT) & NV40_PMC_BACKLIGHT_MASK))
		return 0;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = 31;
	bd = backlight_device_register("nv_backlight", &dev->pdev->dev, dev,
				       &nv40_bl_ops, &props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	dev_priv->backlight = bd;
	bd->props.brightness = nv40_get_intensity(bd);
	backlight_update_status(bd);

	return 0;
}

static int nouveau_nv50_backlight_init(struct drm_device *dev)
{
	struct backlight_properties props;
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	struct backlight_device *bd;

	if (!nv_rd32(dev, NV50_PDISPLAY_SOR_BACKLIGHT))
		return 0;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = 1025;
	bd = backlight_device_register("nv_backlight", &dev->pdev->dev, dev,
				       &nv50_bl_ops, &props);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	dev_priv->backlight = bd;
	bd->props.brightness = nv50_get_intensity(bd);
	backlight_update_status(bd);
	return 0;
}

int nouveau_backlight_init(struct drm_device *dev)
{
	struct drm_nouveau_private *dev_priv = dev->dev_private;

	switch (dev_priv->card_type) {
	case NV_40:
		return nouveau_nv40_backlight_init(dev);
	case NV_50:
		return nouveau_nv50_backlight_init(dev);
	default:
		break;
	}

	return 0;
}

void nouveau_backlight_exit(struct drm_device *dev)
{
	struct drm_nouveau_private *dev_priv = dev->dev_private;

	if (dev_priv->backlight) {
		backlight_device_unregister(dev_priv->backlight);
		dev_priv->backlight = NULL;
	}
}
