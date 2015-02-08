
/* @(#) $Header: /tcpdump/master/tcpdump/rpc_auth.h,v 1.1.2.1 2005/04/27 21:44:07 guy Exp $ (LBL) */


enum sunrpc_auth_stat {
	SUNRPC_AUTH_OK=0,
	/*
	 * failed at remote end
	 */
	SUNRPC_AUTH_BADCRED=1,		/* bogus credentials (seal broken) */
	SUNRPC_AUTH_REJECTEDCRED=2,	/* client should begin new session */
	SUNRPC_AUTH_BADVERF=3,		/* bogus verifier (seal broken) */
	SUNRPC_AUTH_REJECTEDVERF=4,	/* verifier expired or was replayed */
	SUNRPC_AUTH_TOOWEAK=5,		/* rejected due to security reasons */
	/*
	 * failed locally
	*/
	SUNRPC_AUTH_INVALIDRESP=6,	/* bogus response verifier */
	SUNRPC_AUTH_FAILED=7		/* some unknown reason */
};

struct sunrpc_opaque_auth {
	u_int32_t oa_flavor;		/* flavor of auth */
	u_int32_t oa_len;		/* length of opaque body */
	/* zero or more bytes of body */
};

#define SUNRPC_AUTH_NONE	0	/* no authentication */
#define	SUNRPC_AUTH_NULL	0	/* backward compatibility */
#define	SUNRPC_AUTH_UNIX	1	/* unix style (uid, gids) */
#define	SUNRPC_AUTH_SYS		1	/* forward compatibility */
#define	SUNRPC_AUTH_SHORT	2	/* short hand unix style */
#define SUNRPC_AUTH_DES		3	/* des style (encrypted timestamps) */
