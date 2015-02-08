

#ifndef __LINUX_IF_PPPOL2TP_H
#define __LINUX_IF_PPPOL2TP_H

#include <linux/types.h>

#ifdef __KERNEL__
#include <linux/in.h>
#endif

struct pppol2tp_addr {
	__kernel_pid_t	pid;		/* pid that owns the fd.
					 * 0 => current */
	int	fd;			/* FD of UDP socket to use */

	struct sockaddr_in addr;	/* IP address and port to send to */

	__u16 s_tunnel, s_session;	/* For matching incoming packets */
	__u16 d_tunnel, d_session;	/* For sending outgoing packets */
};

struct pppol2tpv3_addr {
	pid_t	pid;			/* pid that owns the fd.
					 * 0 => current */
	int	fd;			/* FD of UDP or IP socket to use */

	struct sockaddr_in addr;	/* IP address and port to send to */

	__u32 s_tunnel, s_session;	/* For matching incoming packets */
	__u32 d_tunnel, d_session;	/* For sending outgoing packets */
};

enum {
	PPPOL2TP_SO_DEBUG	= 1,
	PPPOL2TP_SO_RECVSEQ	= 2,
	PPPOL2TP_SO_SENDSEQ	= 3,
	PPPOL2TP_SO_LNSMODE	= 4,
	PPPOL2TP_SO_REORDERTO	= 5,
};

/* Debug message categories for the DEBUG socket option */
enum {
	PPPOL2TP_MSG_DEBUG	= (1 << 0),	/* verbose debug (if
						 * compiled in) */
	PPPOL2TP_MSG_CONTROL	= (1 << 1),	/* userspace - kernel
						 * interface */
	PPPOL2TP_MSG_SEQ	= (1 << 2),	/* sequence numbers */
	PPPOL2TP_MSG_DATA	= (1 << 3),	/* data packets */
};



#endif
