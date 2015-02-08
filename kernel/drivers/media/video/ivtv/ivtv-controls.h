

#ifndef IVTV_CONTROLS_H
#define IVTV_CONTROLS_H

int ivtv_queryctrl(struct file *file, void *fh, struct v4l2_queryctrl *a);
int ivtv_g_ext_ctrls(struct file *file, void *fh, struct v4l2_ext_controls *a);
int ivtv_s_ext_ctrls(struct file *file, void *fh, struct v4l2_ext_controls *a);
int ivtv_try_ext_ctrls(struct file *file, void *fh, struct v4l2_ext_controls *a);
int ivtv_querymenu(struct file *file, void *fh, struct v4l2_querymenu *a);

#endif
