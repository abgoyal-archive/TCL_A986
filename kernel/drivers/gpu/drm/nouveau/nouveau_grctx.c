

#include <linux/firmware.h>
#include <linux/slab.h>

#include "drmP.h"
#include "nouveau_drv.h"

struct nouveau_ctxprog {
	uint32_t signature;
	uint8_t  version;
	uint16_t length;
	uint32_t data[];
} __attribute__ ((packed));

struct nouveau_ctxvals {
	uint32_t signature;
	uint8_t  version;
	uint32_t length;
	struct {
		uint32_t offset;
		uint32_t value;
	} data[];
} __attribute__ ((packed));

int
nouveau_grctx_prog_load(struct drm_device *dev)
{
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	struct nouveau_pgraph_engine *pgraph = &dev_priv->engine.graph;
	const int chipset = dev_priv->chipset;
	const struct firmware *fw;
	const struct nouveau_ctxprog *cp;
	const struct nouveau_ctxvals *cv;
	char name[32];
	int ret, i;

	if (pgraph->accel_blocked)
		return -ENODEV;

	if (!pgraph->ctxprog) {
		sprintf(name, "nouveau/nv%02x.ctxprog", chipset);
		ret = request_firmware(&fw, name, &dev->pdev->dev);
		if (ret) {
			NV_ERROR(dev, "No ctxprog for NV%02x\n", chipset);
			return ret;
		}

		pgraph->ctxprog = kmemdup(fw->data, fw->size, GFP_KERNEL);
		if (!pgraph->ctxprog) {
			NV_ERROR(dev, "OOM copying ctxprog\n");
			release_firmware(fw);
			return -ENOMEM;
		}

		cp = pgraph->ctxprog;
		if (le32_to_cpu(cp->signature) != 0x5043564e ||
		    cp->version != 0 ||
		    le16_to_cpu(cp->length) != ((fw->size - 7) / 4)) {
			NV_ERROR(dev, "ctxprog invalid\n");
			release_firmware(fw);
			nouveau_grctx_fini(dev);
			return -EINVAL;
		}
		release_firmware(fw);
	}

	if (!pgraph->ctxvals) {
		sprintf(name, "nouveau/nv%02x.ctxvals", chipset);
		ret = request_firmware(&fw, name, &dev->pdev->dev);
		if (ret) {
			NV_ERROR(dev, "No ctxvals for NV%02x\n", chipset);
			nouveau_grctx_fini(dev);
			return ret;
		}

		pgraph->ctxvals = kmemdup(fw->data, fw->size, GFP_KERNEL);
		if (!pgraph->ctxvals) {
			NV_ERROR(dev, "OOM copying ctxvals\n");
			release_firmware(fw);
			nouveau_grctx_fini(dev);
			return -ENOMEM;
		}

		cv = (void *)pgraph->ctxvals;
		if (le32_to_cpu(cv->signature) != 0x5643564e ||
		    cv->version != 0 ||
		    le32_to_cpu(cv->length) != ((fw->size - 9) / 8)) {
			NV_ERROR(dev, "ctxvals invalid\n");
			release_firmware(fw);
			nouveau_grctx_fini(dev);
			return -EINVAL;
		}
		release_firmware(fw);
	}

	cp = pgraph->ctxprog;

	nv_wr32(dev, NV40_PGRAPH_CTXCTL_UCODE_INDEX, 0);
	for (i = 0; i < le16_to_cpu(cp->length); i++)
		nv_wr32(dev, NV40_PGRAPH_CTXCTL_UCODE_DATA,
			le32_to_cpu(cp->data[i]));

	return 0;
}

void
nouveau_grctx_fini(struct drm_device *dev)
{
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	struct nouveau_pgraph_engine *pgraph = &dev_priv->engine.graph;

	if (pgraph->ctxprog) {
		kfree(pgraph->ctxprog);
		pgraph->ctxprog = NULL;
	}

	if (pgraph->ctxvals) {
		kfree(pgraph->ctxprog);
		pgraph->ctxvals = NULL;
	}
}

void
nouveau_grctx_vals_load(struct drm_device *dev, struct nouveau_gpuobj *ctx)
{
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	struct nouveau_pgraph_engine *pgraph = &dev_priv->engine.graph;
	struct nouveau_ctxvals *cv = pgraph->ctxvals;
	int i;

	if (!cv)
		return;

	for (i = 0; i < le32_to_cpu(cv->length); i++)
		nv_wo32(dev, ctx, le32_to_cpu(cv->data[i].offset),
			le32_to_cpu(cv->data[i].value));
}
