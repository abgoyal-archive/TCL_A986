

#ifndef _LINUX_NFSD_CONST_H
#define _LINUX_NFSD_CONST_H

#include <linux/nfs.h>
#include <linux/nfs2.h>
#include <linux/nfs3.h>
#include <linux/nfs4.h>

#define NFSSVC_MAXVERS		3

#define NFSSVC_MAXBLKSIZE	RPCSVC_MAXPAYLOAD
/* NFSv2 is limited by the protocol specification, see RFC 1094 */
#define NFSSVC_MAXBLKSIZE_V2	(8*1024)

#ifdef __KERNEL__

#include <linux/sunrpc/msg_prot.h>

#define NFSD_BUFSIZE		((RPC_MAX_HEADER_WITH_AUTH+26)*XDR_UNIT + NFSSVC_MAXBLKSIZE)

#ifdef CONFIG_NFSD_V4
# define NFSSVC_XDRSIZE		NFS4_SVC_XDRSIZE
#elif defined(CONFIG_NFSD_V3)
# define NFSSVC_XDRSIZE		NFS3_SVC_XDRSIZE
#else
# define NFSSVC_XDRSIZE		NFS2_SVC_XDRSIZE
#endif

#endif /* __KERNEL__ */

#endif /* _LINUX_NFSD_CONST_H */
