

#ifndef V4L2_MEDIABUS_H
#define V4L2_MEDIABUS_H

enum v4l2_mbus_pixelcode {
	V4L2_MBUS_FMT_FIXED = 1,
	V4L2_MBUS_FMT_YUYV8_2X8_LE,
	V4L2_MBUS_FMT_YVYU8_2X8_LE,
	V4L2_MBUS_FMT_YUYV8_2X8_BE,
	V4L2_MBUS_FMT_YVYU8_2X8_BE,
	V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE,
	V4L2_MBUS_FMT_RGB555_2X8_PADHI_BE,
	V4L2_MBUS_FMT_RGB565_2X8_LE,
	V4L2_MBUS_FMT_RGB565_2X8_BE,
	V4L2_MBUS_FMT_SBGGR8_1X8,
	V4L2_MBUS_FMT_SBGGR10_1X10,
	V4L2_MBUS_FMT_GREY8_1X8,
	V4L2_MBUS_FMT_Y10_1X10,
	V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_LE,
	V4L2_MBUS_FMT_SBGGR10_2X8_PADLO_LE,
	V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_BE,
	V4L2_MBUS_FMT_SBGGR10_2X8_PADLO_BE,
	V4L2_MBUS_FMT_SGRBG8_1X8,
};

struct v4l2_mbus_framefmt {
	__u32				width;
	__u32				height;
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_field			field;
	enum v4l2_colorspace		colorspace;
};

static inline void v4l2_fill_pix_format(struct v4l2_pix_format *pix_fmt,
				const struct v4l2_mbus_framefmt *mbus_fmt)
{
	pix_fmt->width = mbus_fmt->width;
	pix_fmt->height = mbus_fmt->height;
	pix_fmt->field = mbus_fmt->field;
	pix_fmt->colorspace = mbus_fmt->colorspace;
}

static inline void v4l2_fill_mbus_format(struct v4l2_mbus_framefmt *mbus_fmt,
			   const struct v4l2_pix_format *pix_fmt,
			   enum v4l2_mbus_pixelcode code)
{
	mbus_fmt->width = pix_fmt->width;
	mbus_fmt->height = pix_fmt->height;
	mbus_fmt->field = pix_fmt->field;
	mbus_fmt->colorspace = pix_fmt->colorspace;
	mbus_fmt->code = code;
}

#endif
