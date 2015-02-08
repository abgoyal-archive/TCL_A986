
/* @(#) $Header: /tcpdump/master/tcpdump/pmap_prot.h,v 1.1.2.2 2005/04/27 21:44:06 guy Exp $ (LBL) */


#define SUNRPC_PMAPPORT		((u_int16_t)111)
#define SUNRPC_PMAPPROG		((u_int32_t)100000)
#define SUNRPC_PMAPVERS		((u_int32_t)2)
#define SUNRPC_PMAPVERS_PROTO	((u_int32_t)2)
#define SUNRPC_PMAPVERS_ORIG	((u_int32_t)1)
#define SUNRPC_PMAPPROC_NULL	((u_int32_t)0)
#define SUNRPC_PMAPPROC_SET	((u_int32_t)1)
#define SUNRPC_PMAPPROC_UNSET	((u_int32_t)2)
#define SUNRPC_PMAPPROC_GETPORT	((u_int32_t)3)
#define SUNRPC_PMAPPROC_DUMP	((u_int32_t)4)
#define SUNRPC_PMAPPROC_CALLIT	((u_int32_t)5)

struct sunrpc_pmap {
	u_int32_t pm_prog;
	u_int32_t pm_vers;
	u_int32_t pm_prot;
	u_int32_t pm_port;
};
