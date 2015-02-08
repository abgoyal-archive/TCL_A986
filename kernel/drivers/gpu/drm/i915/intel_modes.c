

#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include "drmP.h"
#include "intel_drv.h"
#include "i915_drv.h"

bool intel_ddc_probe(struct intel_encoder *intel_encoder)
{
	u8 out_buf[] = { 0x0, 0x0};
	u8 buf[2];
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = 0x50,
			.flags = 0,
			.len = 1,
			.buf = out_buf,
		},
		{
			.addr = 0x50,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = buf,
		}
	};

	intel_i2c_quirk_set(intel_encoder->enc.dev, true);
	ret = i2c_transfer(intel_encoder->ddc_bus, msgs, 2);
	intel_i2c_quirk_set(intel_encoder->enc.dev, false);
	if (ret == 2)
		return true;

	return false;
}

int intel_ddc_get_modes(struct drm_connector *connector,
			struct i2c_adapter *adapter)
{
	struct edid *edid;
	int ret = 0;

	intel_i2c_quirk_set(connector->dev, true);
	edid = drm_get_edid(connector, adapter);
	intel_i2c_quirk_set(connector->dev, false);
	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		connector->display_info.raw_edid = NULL;
		kfree(edid);
	}

	return ret;
}
