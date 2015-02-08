

#ifndef _ASM_VPE_H
#define _ASM_VPE_H

struct vpe_notifications {
	void (*start)(int vpe);
	void (*stop)(int vpe);

	struct list_head list;
};


extern int vpe_notify(int index, struct vpe_notifications *notify);

extern void *vpe_get_shared(int index);
extern int vpe_getuid(int index);
extern int vpe_getgid(int index);
extern char *vpe_getcwd(int index);

#endif /* _ASM_VPE_H */
