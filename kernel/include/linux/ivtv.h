

#ifndef __LINUX_IVTV_H__
#define __LINUX_IVTV_H__

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/videodev2.h>


struct ivtv_dma_frame {
	enum v4l2_buf_type type; /* V4L2_BUF_TYPE_VIDEO_OUTPUT */
	__u32 pixelformat;	 /* 0 == same as destination */
	void __user *y_source;   /* if NULL and type == V4L2_BUF_TYPE_VIDEO_OUTPUT,
				    then just switch to user DMA YUV output mode */
	void __user *uv_source;  /* Unused for RGB pixelformats */
	struct v4l2_rect src;
	struct v4l2_rect dst;
	__u32 src_width;
	__u32 src_height;
};

#define IVTV_IOC_DMA_FRAME  _IOW ('V', BASE_VIDIOC_PRIVATE+0, struct ivtv_dma_frame)

/* Deprecated defines: applications should use the defines from videodev2.h */
#define IVTV_SLICED_TYPE_TELETEXT_B     V4L2_MPEG_VBI_IVTV_TELETEXT_B
#define IVTV_SLICED_TYPE_CAPTION_525    V4L2_MPEG_VBI_IVTV_CAPTION_525
#define IVTV_SLICED_TYPE_WSS_625        V4L2_MPEG_VBI_IVTV_WSS_625
#define IVTV_SLICED_TYPE_VPS            V4L2_MPEG_VBI_IVTV_VPS

#endif /* _LINUX_IVTV_H */
