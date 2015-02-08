

int cx18_queryctrl(struct file *file, void *fh, struct v4l2_queryctrl *a);
int cx18_g_ext_ctrls(struct file *file, void *fh, struct v4l2_ext_controls *a);
int cx18_s_ext_ctrls(struct file *file, void *fh, struct v4l2_ext_controls *a);
int cx18_try_ext_ctrls(struct file *file, void *fh,
			struct v4l2_ext_controls *a);
int cx18_querymenu(struct file *file, void *fh, struct v4l2_querymenu *a);
