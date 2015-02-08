
#ifndef _ASM_POWERPC_IPCBUF_H
#define _ASM_POWERPC_IPCBUF_H


#include <linux/types.h>

struct ipc64_perm
{
	__kernel_key_t	key;
	__kernel_uid_t	uid;
	__kernel_gid_t	gid;
	__kernel_uid_t	cuid;
	__kernel_gid_t	cgid;
	__kernel_mode_t	mode;
	unsigned int	seq;
	unsigned int	__pad1;
	unsigned long long __unused1;
	unsigned long long __unused2;
};

#endif /* _ASM_POWERPC_IPCBUF_H */
