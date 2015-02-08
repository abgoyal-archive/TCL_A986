

#ifndef _LINUX_TIPC_H_
#define _LINUX_TIPC_H_

#include <linux/types.h>

 
struct tipc_portid {
	__u32 ref;
	__u32 node;
};

struct tipc_name {
	__u32 type;
	__u32 instance;
};

struct tipc_name_seq {
	__u32 type;
	__u32 lower;
	__u32 upper;
};

static inline __u32 tipc_addr(unsigned int zone,
			      unsigned int cluster,
			      unsigned int node)
{
	return (zone << 24) | (cluster << 12) | node;
}

static inline unsigned int tipc_zone(__u32 addr)
{
	return addr >> 24;
}

static inline unsigned int tipc_cluster(__u32 addr)
{
	return (addr >> 12) & 0xfff;
}

static inline unsigned int tipc_node(__u32 addr)
{
	return addr & 0xfff;
}


#define TIPC_CFG_SRV		0	/* configuration service name type */
#define TIPC_TOP_SRV		1	/* topology service name type */
#define TIPC_RESERVED_TYPES	64	/* lowest user-publishable name type */


#define TIPC_ZONE_SCOPE		1
#define TIPC_CLUSTER_SCOPE	2
#define TIPC_NODE_SCOPE		3


#define TIPC_MAX_USER_MSG_SIZE	66000


#define TIPC_LOW_IMPORTANCE		0
#define TIPC_MEDIUM_IMPORTANCE		1
#define TIPC_HIGH_IMPORTANCE		2
#define TIPC_CRITICAL_IMPORTANCE	3


#define TIPC_OK			0
#define TIPC_ERR_NO_NAME	1
#define TIPC_ERR_NO_PORT	2
#define TIPC_ERR_NO_NODE	3
#define TIPC_ERR_OVERLOAD	4
#define TIPC_CONN_SHUTDOWN	5


#define TIPC_SUB_SERVICE     	0x00  	/* Filter for service availability    */
#define TIPC_SUB_PORTS     	0x01  	/* Filter for port availability  */
#define TIPC_SUB_CANCEL         0x04    /* Cancel a subscription         */

#define TIPC_WAIT_FOREVER	~0	/* timeout for permanent subscription */

struct tipc_subscr {
	struct tipc_name_seq seq;	/* NBO. Name sequence of interest */
	__u32 timeout;			/* NBO. Subscription duration (in ms) */
        __u32 filter;   		/* NBO. Bitmask of filter options */
	char usr_handle[8];		/* Opaque. Available for subscriber use */
};

#define TIPC_PUBLISHED		1	/* publication event */
#define TIPC_WITHDRAWN		2	/* withdraw event */
#define TIPC_SUBSCR_TIMEOUT	3	/* subscription timeout event */

struct tipc_event {
	__u32 event;			/* NBO. Event type, as defined above */
	__u32 found_lower;		/* NBO. Matching name seq instances  */
	__u32 found_upper;		/*  "      "       "   "    "        */
	struct tipc_portid port;	/* NBO. Associated port              */
	struct tipc_subscr s;		/* Original, associated subscription */
};


#ifndef AF_TIPC
#define AF_TIPC		30
#endif

#ifndef PF_TIPC
#define PF_TIPC		AF_TIPC
#endif

#ifndef SOL_TIPC
#define SOL_TIPC	271
#endif

#define TIPC_ADDR_NAMESEQ	1
#define TIPC_ADDR_MCAST		1
#define TIPC_ADDR_NAME		2
#define TIPC_ADDR_ID		3

struct sockaddr_tipc {
	unsigned short family;
	unsigned char  addrtype;
	signed   char  scope;
	union {
		struct tipc_portid id;
		struct tipc_name_seq nameseq;
		struct {
			struct tipc_name name;
			__u32 domain;
		} name;
	} addr;
};


#define TIPC_ERRINFO	1	/* error info */
#define TIPC_RETDATA	2	/* returned data */
#define TIPC_DESTNAME	3	/* destination name */


#define TIPC_IMPORTANCE		127	/* Default: TIPC_LOW_IMPORTANCE */
#define TIPC_SRC_DROPPABLE	128	/* Default: based on socket type */
#define TIPC_DEST_DROPPABLE	129	/* Default: based on socket type */
#define TIPC_CONN_TIMEOUT	130	/* Default: 8000 (ms)  */
#define TIPC_NODE_RECVQ_DEPTH	131	/* Default: none (read only) */
#define TIPC_SOCK_RECVQ_DEPTH	132	/* Default: none (read only) */

#endif
