

#ifndef SOC_MEDIABUS_H
#define SOC_MEDIABUS_H

#include <linux/videodev2.h>

#include <media/v4l2-mediabus.h>

enum soc_mbus_packing {
	SOC_MBUS_PACKING_NONE,
	SOC_MBUS_PACKING_2X8_PADHI,
	SOC_MBUS_PACKING_2X8_PADLO,
	SOC_MBUS_PACKING_EXTEND16,
};

enum soc_mbus_order {
	SOC_MBUS_ORDER_LE,
	SOC_MBUS_ORDER_BE,
};

struct soc_mbus_pixelfmt {
	const char		*name;
	u32			fourcc;
	enum soc_mbus_packing	packing;
	enum soc_mbus_order	order;
	u8			bits_per_sample;
};

const struct soc_mbus_pixelfmt *soc_mbus_get_fmtdesc(
	enum v4l2_mbus_pixelcode code);
s32 soc_mbus_bytes_per_line(u32 width, const struct soc_mbus_pixelfmt *mf);

#endif
