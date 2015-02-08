

#ifndef V4L2_FH_H
#define V4L2_FH_H

#include <linux/list.h>

struct video_device;
struct v4l2_events;

struct v4l2_fh {
	struct list_head	list;
	struct video_device	*vdev;
	struct v4l2_events      *events; /* events, pending and subscribed */
};

int v4l2_fh_init(struct v4l2_fh *fh, struct video_device *vdev);
void v4l2_fh_add(struct v4l2_fh *fh);
void v4l2_fh_del(struct v4l2_fh *fh);
void v4l2_fh_exit(struct v4l2_fh *fh);

#endif /* V4L2_EVENT_H */
