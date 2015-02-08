
#ifndef _ALPHA_IPCBUF_H
#define _ALPHA_IPCBUF_H


struct ipc64_perm
{
	__kernel_key_t	key;
	__kernel_uid_t	uid;
	__kernel_gid_t	gid;
	__kernel_uid_t	cuid;
	__kernel_gid_t	cgid;
	__kernel_mode_t	mode; 
	unsigned short	seq;
	unsigned short	__pad1;
	unsigned long	__unused1;
	unsigned long	__unused2;
};

#endif /* _ALPHA_IPCBUF_H */
