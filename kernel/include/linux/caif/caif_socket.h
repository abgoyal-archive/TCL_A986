

#ifndef _LINUX_CAIF_SOCKET_H
#define _LINUX_CAIF_SOCKET_H

#include <linux/types.h>

#ifdef __KERNEL__
#include <linux/socket.h>
#else
#include <sys/socket.h>
#endif

enum caif_link_selector {
	CAIF_LINK_HIGH_BANDW,
	CAIF_LINK_LOW_LATENCY
};

enum caif_channel_priority {
	CAIF_PRIO_MIN	 = 0x01,
	CAIF_PRIO_LOW	 = 0x04,
	CAIF_PRIO_NORMAL = 0x0f,
	CAIF_PRIO_HIGH	 = 0x14,
	CAIF_PRIO_MAX	 = 0x1F
};

enum caif_protocol_type {
	CAIFPROTO_AT,
	CAIFPROTO_DATAGRAM,
	CAIFPROTO_DATAGRAM_LOOP,
	CAIFPROTO_UTIL,
	CAIFPROTO_RFM,
	_CAIFPROTO_MAX
};
#define	CAIFPROTO_MAX _CAIFPROTO_MAX

enum caif_at_type {
	CAIF_ATTYPE_PLAIN = 2
};

struct sockaddr_caif {
	sa_family_t  family;
	union {
		struct {
			__u8  type;		/* type: enum caif_at_type */
		} at;				/* CAIFPROTO_AT */
		struct {
			char	  service[16];
		} util;				/* CAIFPROTO_UTIL */
		union {
			__u32 connection_id;
			__u8  nsapi;
		} dgm;				/* CAIFPROTO_DATAGRAM(_LOOP)*/
		struct {
			__u32 connection_id;
			char	  volume[16];
		} rfm;				/* CAIFPROTO_RFM */
	} u;
};

enum caif_socket_opts {
	CAIFSO_LINK_SELECT	= 127,
	CAIFSO_REQ_PARAM	= 128,
	CAIFSO_RSP_PARAM	= 129,
};

#endif /* _LINUX_CAIF_SOCKET_H */
