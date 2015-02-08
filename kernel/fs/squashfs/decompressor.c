

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/buffer_head.h>

#include "squashfs_fs.h"
#include "squashfs_fs_sb.h"
#include "squashfs_fs_i.h"
#include "decompressor.h"
#include "squashfs.h"


static const struct squashfs_decompressor squashfs_lzma_unsupported_comp_ops = {
	NULL, NULL, NULL, LZMA_COMPRESSION, "lzma", 0
};

static const struct squashfs_decompressor squashfs_lzo_unsupported_comp_ops = {
	NULL, NULL, NULL, LZO_COMPRESSION, "lzo", 0
};

static const struct squashfs_decompressor squashfs_unknown_comp_ops = {
	NULL, NULL, NULL, 0, "unknown", 0
};

static const struct squashfs_decompressor *decompressor[] = {
	&squashfs_zlib_comp_ops,
	&squashfs_lzma_unsupported_comp_ops,
	&squashfs_lzo_unsupported_comp_ops,
	&squashfs_unknown_comp_ops
};


const struct squashfs_decompressor *squashfs_lookup_decompressor(int id)
{
	int i;

	for (i = 0; decompressor[i]->id; i++)
		if (id == decompressor[i]->id)
			break;

	return decompressor[i];
}
