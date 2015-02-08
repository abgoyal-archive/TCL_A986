

#ifndef _XTENSA_IPCBUF_H
#define _XTENSA_IPCBUF_H


struct ipc64_perm
{
	__kernel_key_t		key;
	__kernel_uid32_t	uid;
	__kernel_gid32_t	gid;
	__kernel_uid32_t	cuid;
	__kernel_gid32_t	cgid;
	__kernel_mode_t		mode;
	unsigned long		seq;
	unsigned long		__unused1;
	unsigned long		__unused2;
};

#endif /* _XTENSA_IPCBUF_H */
