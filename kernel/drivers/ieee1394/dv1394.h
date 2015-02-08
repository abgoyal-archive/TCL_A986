

#ifndef _DV_1394_H
#define _DV_1394_H

/* This is the public user-space interface. Try not to break it. */

#define DV1394_API_VERSION 0x20011127



/* maximum number of frames in the ringbuffer */
#define DV1394_MAX_FRAMES 32

/* number of *full* isochronous packets per DV frame */
#define DV1394_NTSC_PACKETS_PER_FRAME 250
#define DV1394_PAL_PACKETS_PER_FRAME  300

/* size of one frame's worth of DV data, in bytes */
#define DV1394_NTSC_FRAME_SIZE (480 * DV1394_NTSC_PACKETS_PER_FRAME)
#define DV1394_PAL_FRAME_SIZE  (480 * DV1394_PAL_PACKETS_PER_FRAME)


/* ioctl() commands */
#include "ieee1394-ioctl.h"


enum pal_or_ntsc {
	DV1394_NTSC = 0,
	DV1394_PAL
};




/* this is the argument to DV1394_INIT */
struct dv1394_init {
	/* DV1394_API_VERSION */
	unsigned int api_version;

	/* isochronous transmission channel to use */
	unsigned int channel;

	/* number of frames in the ringbuffer. Must be at least 2
	   and at most DV1394_MAX_FRAMES. */
	unsigned int n_frames;

	/* send/receive PAL or NTSC video format */
	enum pal_or_ntsc format;

	/* the following are used only for transmission */

	/* set these to zero unless you want a
	   non-default empty packet rate (see below) */
	unsigned long cip_n;
	unsigned long cip_d;

	/* set this to zero unless you want a
	   non-default SYT cycle offset (default = 3 cycles) */
	unsigned int syt_offset;
};


/* Q: What are cip_n and cip_d? */




struct dv1394_status {
	/* this embedded init struct returns the current dv1394
	   parameters in use */
	struct dv1394_init init;

	/* the ringbuffer frame that is currently being
	   displayed. (-1 if the device is not transmitting anything) */
	int active_frame;

	/* index of the first buffer (ahead of active_frame) that
	   is ready to be filled with data */
	unsigned int first_clear_frame;

	/* how many buffers, including first_clear_buffer, are
	   ready to be filled with data */
	unsigned int n_clear_frames;

	/* how many times the DV stream has underflowed, overflowed,
	   or otherwise encountered an error, since the previous call
	   to DV1394_GET_STATUS */
	unsigned int dropped_frames;

	/* N.B. The dropped_frames counter is only a lower bound on the actual
	   number of dropped frames, with the special case that if dropped_frames
	   is zero, then it is guaranteed that NO frames have been dropped
	   since the last call to DV1394_GET_STATUS.
	*/
};


#endif /* _DV_1394_H */
