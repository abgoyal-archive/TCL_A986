

#ifndef _MEYE_H_
#define _MEYE_H_

/****************************************************************************/
/* Private API for handling mjpeg capture / playback.                       */
/****************************************************************************/

struct meye_params {
	unsigned char subsample;
	unsigned char quality;
	unsigned char sharpness;
	unsigned char agc;
	unsigned char picture;
	unsigned char framerate;
};

/* query the extended parameters */
#define MEYEIOC_G_PARAMS	_IOR ('v', BASE_VIDIOC_PRIVATE+0, struct meye_params)
/* set the extended parameters */
#define MEYEIOC_S_PARAMS	_IOW ('v', BASE_VIDIOC_PRIVATE+1, struct meye_params)
/* queue a buffer for mjpeg capture */
#define MEYEIOC_QBUF_CAPT	_IOW ('v', BASE_VIDIOC_PRIVATE+2, int)
/* sync a previously queued mjpeg buffer */
#define MEYEIOC_SYNC		_IOWR('v', BASE_VIDIOC_PRIVATE+3, int)
/* get a still uncompressed snapshot */
#define MEYEIOC_STILLCAPT	_IO  ('v', BASE_VIDIOC_PRIVATE+4)
/* get a jpeg compressed snapshot */
#define MEYEIOC_STILLJCAPT	_IOR ('v', BASE_VIDIOC_PRIVATE+5, int)

/* V4L2 private controls */
#define V4L2_CID_AGC		V4L2_CID_PRIVATE_BASE
#define V4L2_CID_MEYE_SHARPNESS	(V4L2_CID_PRIVATE_BASE + 1)
#define V4L2_CID_PICTURE	(V4L2_CID_PRIVATE_BASE + 2)
#define V4L2_CID_JPEGQUAL	(V4L2_CID_PRIVATE_BASE + 3)
#define V4L2_CID_FRAMERATE	(V4L2_CID_PRIVATE_BASE + 4)

#endif
