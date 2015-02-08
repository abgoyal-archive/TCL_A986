
#ifndef __LINUX_MSM_RPCROUTER_H
#define __LINUX_MSM_RPCROUTER_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define RPC_ROUTER_VERSION_V1 0x00010000

struct rpcrouter_ioctl_server_args {
	uint32_t prog;
	uint32_t vers;
};

#define RPC_ROUTER_IOCTL_MAGIC (0xC1)

#define RPC_ROUTER_IOCTL_GET_VERSION \
	_IOR(RPC_ROUTER_IOCTL_MAGIC, 0, unsigned int)

#define RPC_ROUTER_IOCTL_GET_MTU \
	_IOR(RPC_ROUTER_IOCTL_MAGIC, 1, unsigned int)

#define RPC_ROUTER_IOCTL_REGISTER_SERVER \
	_IOWR(RPC_ROUTER_IOCTL_MAGIC, 2, unsigned int)

#define RPC_ROUTER_IOCTL_UNREGISTER_SERVER \
	_IOWR(RPC_ROUTER_IOCTL_MAGIC, 3, unsigned int)

#define RPC_ROUTER_IOCTL_GET_MINOR_VERSION \
	_IOW(RPC_ROUTER_IOCTL_MAGIC, 4, unsigned int)

#endif
