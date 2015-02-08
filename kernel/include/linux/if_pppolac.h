

#ifndef __LINUX_IF_PPPOLAC_H
#define __LINUX_IF_PPPOLAC_H

#include <linux/socket.h>
#include <linux/types.h>

#define PX_PROTO_OLAC	2

struct sockaddr_pppolac {
	sa_family_t	sa_family;	/* AF_PPPOX */
	unsigned int	sa_protocol;	/* PX_PROTO_OLAC */
	int		udp_socket;
	struct __attribute__((packed)) {
		__u16	tunnel, session;
	} local, remote;
} __attribute__((packed));

#endif /* __LINUX_IF_PPPOLAC_H */
