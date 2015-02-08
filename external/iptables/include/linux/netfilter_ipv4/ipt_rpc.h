

#ifndef _IPT_RPC_H
#define _IPT_RPC_H

struct ipt_rpc_data;

struct ipt_rpc_info {
	int inverse;
	int strict;
	const char c_procs[1408];
	int i_procs;
	struct ipt_rpc_data *data;
};

#endif /* _IPT_RPC_H */
