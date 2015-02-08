
/* @(#) $Header: /tcpdump/master/tcpdump/rpc_msg.h,v 1.1.2.1 2005/04/27 21:44:07 guy Exp $ (LBL) */


#define SUNRPC_MSG_VERSION	((u_int32_t) 2)


enum sunrpc_msg_type {
	SUNRPC_CALL=0,
	SUNRPC_REPLY=1
};

enum sunrpc_reply_stat {
	SUNRPC_MSG_ACCEPTED=0,
	SUNRPC_MSG_DENIED=1
};

enum sunrpc_accept_stat {
	SUNRPC_SUCCESS=0,
	SUNRPC_PROG_UNAVAIL=1,
	SUNRPC_PROG_MISMATCH=2,
	SUNRPC_PROC_UNAVAIL=3,
	SUNRPC_GARBAGE_ARGS=4,
	SUNRPC_SYSTEM_ERR=5
};

enum sunrpc_reject_stat {
	SUNRPC_RPC_MISMATCH=0,
	SUNRPC_AUTH_ERROR=1
};


struct sunrpc_rejected_reply {
	u_int32_t		 rj_stat;	/* enum reject_stat */
	union {
		struct {
			u_int32_t low;
			u_int32_t high;
		} RJ_versions;
		u_int32_t RJ_why;  /* enum auth_stat - why authentication did not work */
	} ru;
#define	rj_vers	ru.RJ_versions
#define	rj_why	ru.RJ_why
};

struct sunrpc_reply_body {
	u_int32_t	rp_stat;		/* enum reply_stat */
	struct sunrpc_rejected_reply rp_reject;	/* if rejected */
};

struct sunrpc_call_body {
	u_int32_t cb_rpcvers;	/* must be equal to two */
	u_int32_t cb_prog;
	u_int32_t cb_vers;
	u_int32_t cb_proc;
	struct sunrpc_opaque_auth cb_cred;
	/* followed by opaque verifier */
};

struct sunrpc_msg {
	u_int32_t		rm_xid;
	u_int32_t		rm_direction;	/* enum msg_type */
	union {
		struct sunrpc_call_body RM_cmb;
		struct sunrpc_reply_body RM_rmb;
	} ru;
#define	rm_call		ru.RM_cmb
#define	rm_reply	ru.RM_rmb
};
#define	acpted_rply	ru.RM_rmb.ru.RP_ar
#define	rjcted_rply	ru.RM_rmb.ru.RP_dr
