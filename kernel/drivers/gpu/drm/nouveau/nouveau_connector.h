

#ifndef __NOUVEAU_CONNECTOR_H__
#define __NOUVEAU_CONNECTOR_H__

#include "drm_edid.h"
#include "nouveau_i2c.h"

struct nouveau_connector {
	struct drm_connector base;

	struct dcb_connector_table_entry *dcb;

	int scaling_mode;
	bool use_dithering;

	struct nouveau_encoder *detected_encoder;
	struct edid *edid;
	struct drm_display_mode *native_mode;
};

static inline struct nouveau_connector *nouveau_connector(
						struct drm_connector *con)
{
	return container_of(con, struct nouveau_connector, base);
}

int nouveau_connector_create(struct drm_device *,
			     struct dcb_connector_table_entry *);

#endif /* __NOUVEAU_CONNECTOR_H__ */
