
#ifndef _LINUX_NFS4_MOUNT_H
#define _LINUX_NFS4_MOUNT_H


#define NFS4_MOUNT_VERSION	1

struct nfs_string {
	unsigned int len;
	const char __user * data;
};

struct nfs4_mount_data {
	int version;				/* 1 */
	int flags;				/* 1 */
	int rsize;				/* 1 */
	int wsize;				/* 1 */
	int timeo;				/* 1 */
	int retrans;				/* 1 */
	int acregmin;				/* 1 */
	int acregmax;				/* 1 */
	int acdirmin;				/* 1 */
	int acdirmax;				/* 1 */

	/* see the definition of 'struct clientaddr4' in RFC3010 */
	struct nfs_string client_addr;		/* 1 */

	/* Mount path */
	struct nfs_string mnt_path;		/* 1 */

	/* Server details */
	struct nfs_string hostname;		/* 1 */
	/* Server IP address */
	unsigned int host_addrlen;		/* 1 */
	struct sockaddr __user * host_addr;	/* 1 */

	/* Transport protocol to use */
	int proto;				/* 1 */

	/* Pseudo-flavours to use for authentication. See RFC2623 */
	int auth_flavourlen;			/* 1 */
	int __user *auth_flavours;		/* 1 */
};

/* bits in the flags field */

#define NFS4_MOUNT_SOFT		0x0001	/* 1 */
#define NFS4_MOUNT_INTR		0x0002	/* 1 */
#define NFS4_MOUNT_NOCTO	0x0010	/* 1 */
#define NFS4_MOUNT_NOAC		0x0020	/* 1 */
#define NFS4_MOUNT_STRICTLOCK	0x1000	/* 1 */
#define NFS4_MOUNT_UNSHARED	0x8000	/* 1 */
#define NFS4_MOUNT_FLAGMASK	0x9033

#endif
