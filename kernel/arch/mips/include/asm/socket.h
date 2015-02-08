
#ifndef _ASM_SOCKET_H
#define _ASM_SOCKET_H

#include <asm/sockios.h>

#define SOL_SOCKET	0xffff

#define SO_DEBUG	0x0001	/* Record debugging information.  */
#define SO_REUSEADDR	0x0004	/* Allow reuse of local addresses.  */
#define SO_KEEPALIVE	0x0008	/* Keep connections alive and send
				   SIGPIPE when they die.  */
#define SO_DONTROUTE	0x0010	/* Don't do local routing.  */
#define SO_BROADCAST	0x0020	/* Allow transmission of
				   broadcast messages.  */
#define SO_LINGER	0x0080	/* Block on close of a reliable
				   socket to transmit pending data.  */
#define SO_OOBINLINE 0x0100	/* Receive out-of-band data in-band.  */
#if 0
To add: #define SO_REUSEPORT 0x0200	/* Allow local address and port reuse.  */
#endif

#define SO_TYPE		0x1008	/* Compatible name for SO_STYLE.  */
#define SO_STYLE	SO_TYPE	/* Synonym */
#define SO_ERROR	0x1007	/* get error status and clear */
#define SO_SNDBUF	0x1001	/* Send buffer size. */
#define SO_RCVBUF	0x1002	/* Receive buffer. */
#define SO_SNDLOWAT	0x1003	/* send low-water mark */
#define SO_RCVLOWAT	0x1004	/* receive low-water mark */
#define SO_SNDTIMEO	0x1005	/* send timeout */
#define SO_RCVTIMEO 	0x1006	/* receive timeout */
#define SO_ACCEPTCONN	0x1009
#define SO_PROTOCOL	0x1028	/* protocol type */
#define SO_DOMAIN	0x1029	/* domain/socket family */

/* linux-specific, might as well be the same as on i386 */
#define SO_NO_CHECK	11
#define SO_PRIORITY	12
#define SO_BSDCOMPAT	14

#define SO_PASSCRED	17
#define SO_PEERCRED	18

/* Security levels - as per NRL IPv6 - don't actually do anything */
#define SO_SECURITY_AUTHENTICATION		22
#define SO_SECURITY_ENCRYPTION_TRANSPORT	23
#define SO_SECURITY_ENCRYPTION_NETWORK		24

#define SO_BINDTODEVICE		25

/* Socket filtering */
#define SO_ATTACH_FILTER        26
#define SO_DETACH_FILTER        27

#define SO_PEERNAME             28
#define SO_TIMESTAMP		29
#define SCM_TIMESTAMP		SO_TIMESTAMP

#define SO_PEERSEC		30
#define SO_SNDBUFFORCE		31
#define SO_RCVBUFFORCE		33
#define SO_PASSSEC		34
#define SO_TIMESTAMPNS		35
#define SCM_TIMESTAMPNS		SO_TIMESTAMPNS

#define SO_MARK			36

#define SO_TIMESTAMPING		37
#define SCM_TIMESTAMPING	SO_TIMESTAMPING

#define SO_RXQ_OVFL             40

#ifdef __KERNEL__

enum sock_type {
	SOCK_DGRAM	= 1,
	SOCK_STREAM	= 2,
	SOCK_RAW	= 3,
	SOCK_RDM	= 4,
	SOCK_SEQPACKET	= 5,
	SOCK_DCCP	= 6,
	SOCK_PACKET	= 10,
};

#define SOCK_MAX (SOCK_PACKET + 1)
#define SOCK_TYPE_MASK 0xf

/* Flags for socket, socketpair, paccept */
#define SOCK_CLOEXEC	O_CLOEXEC
#define SOCK_NONBLOCK	O_NONBLOCK

#define ARCH_HAS_SOCKET_TYPES 1

#endif /* __KERNEL__ */

#endif /* _ASM_SOCKET_H */
