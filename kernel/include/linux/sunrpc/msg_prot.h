

#ifndef _LINUX_SUNRPC_MSGPROT_H_
#define _LINUX_SUNRPC_MSGPROT_H_

#ifdef __KERNEL__ /* user programs should get these from the rpc header files */

#define RPC_VERSION 2

/* size of an XDR encoding unit in bytes, i.e. 32bit */
#define XDR_UNIT	(4)

/* spec defines authentication flavor as an unsigned 32 bit integer */
typedef u32	rpc_authflavor_t;

enum rpc_auth_flavors {
	RPC_AUTH_NULL  = 0,
	RPC_AUTH_UNIX  = 1,
	RPC_AUTH_SHORT = 2,
	RPC_AUTH_DES   = 3,
	RPC_AUTH_KRB   = 4,
	RPC_AUTH_GSS   = 6,
	RPC_AUTH_MAXFLAVOR = 8,
	/* pseudoflavors: */
	RPC_AUTH_GSS_KRB5  = 390003,
	RPC_AUTH_GSS_KRB5I = 390004,
	RPC_AUTH_GSS_KRB5P = 390005,
	RPC_AUTH_GSS_LKEY  = 390006,
	RPC_AUTH_GSS_LKEYI = 390007,
	RPC_AUTH_GSS_LKEYP = 390008,
	RPC_AUTH_GSS_SPKM  = 390009,
	RPC_AUTH_GSS_SPKMI = 390010,
	RPC_AUTH_GSS_SPKMP = 390011,
};

/* Maximum size (in bytes) of an rpc credential or verifier */
#define RPC_MAX_AUTH_SIZE (400)

enum rpc_msg_type {
	RPC_CALL = 0,
	RPC_REPLY = 1
};

enum rpc_reply_stat {
	RPC_MSG_ACCEPTED = 0,
	RPC_MSG_DENIED = 1
};

enum rpc_accept_stat {
	RPC_SUCCESS = 0,
	RPC_PROG_UNAVAIL = 1,
	RPC_PROG_MISMATCH = 2,
	RPC_PROC_UNAVAIL = 3,
	RPC_GARBAGE_ARGS = 4,
	RPC_SYSTEM_ERR = 5,
	/* internal use only */
	RPC_DROP_REPLY = 60000,
};

enum rpc_reject_stat {
	RPC_MISMATCH = 0,
	RPC_AUTH_ERROR = 1
};

enum rpc_auth_stat {
	RPC_AUTH_OK = 0,
	RPC_AUTH_BADCRED = 1,
	RPC_AUTH_REJECTEDCRED = 2,
	RPC_AUTH_BADVERF = 3,
	RPC_AUTH_REJECTEDVERF = 4,
	RPC_AUTH_TOOWEAK = 5,
	/* RPCSEC_GSS errors */
	RPCSEC_GSS_CREDPROBLEM = 13,
	RPCSEC_GSS_CTXPROBLEM = 14
};

#define RPC_MAXNETNAMELEN	256


typedef __be32	rpc_fraghdr;

#define	RPC_LAST_STREAM_FRAGMENT	(1U << 31)
#define	RPC_FRAGMENT_SIZE_MASK		(~RPC_LAST_STREAM_FRAGMENT)
#define	RPC_MAX_FRAGMENT_SIZE		((1U << 31) - 1)

#define RPC_CALLHDRSIZE		(6)
#define RPC_REPHDRSIZE		(4)


#define RPC_MAX_HEADER_WITH_AUTH \
	(RPC_CALLHDRSIZE + 2*(2+RPC_MAX_AUTH_SIZE/4))

#define RPCBIND_NETID_UDP	"udp"
#define RPCBIND_NETID_TCP	"tcp"
#define RPCBIND_NETID_UDP6	"udp6"
#define RPCBIND_NETID_TCP6	"tcp6"

#define RPCBIND_MAXNETIDLEN	(4u)


#include <linux/inet.h>

/* Maximum size of the port number part of a universal address */
#define RPCBIND_MAXUADDRPLEN	sizeof(".255.255")

/* Maximum size of an IPv4 universal address */
#define RPCBIND_MAXUADDR4LEN	\
		(INET_ADDRSTRLEN + RPCBIND_MAXUADDRPLEN)

/* Maximum size of an IPv6 universal address */
#define RPCBIND_MAXUADDR6LEN	\
		(INET6_ADDRSTRLEN + RPCBIND_MAXUADDRPLEN)

/* Assume INET6_ADDRSTRLEN will always be larger than INET_ADDRSTRLEN... */
#define RPCBIND_MAXUADDRLEN	RPCBIND_MAXUADDR6LEN

#endif /* __KERNEL__ */
#endif /* _LINUX_SUNRPC_MSGPROT_H_ */
