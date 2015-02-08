

/* Written 1997-2000 by Werner Almesberger, EPFL LRC/ICA */


#ifndef LINUX_ATM_TCP_H
#define LINUX_ATM_TCP_H

#include <linux/atmapi.h>
#include <linux/atm.h>
#include <linux/atmioc.h>
#include <linux/types.h>



struct atmtcp_hdr {
	__u16	vpi;
	__u16	vci;
	__u32	length;		/* ... of data part */
};


#define ATMTCP_HDR_MAGIC	(~0)	/* this length indicates a command */
#define ATMTCP_CTRL_OPEN	1	/* request/reply */
#define ATMTCP_CTRL_CLOSE	2	/* request/reply */

struct atmtcp_control {
	struct atmtcp_hdr hdr;	/* must be first */
	int type;		/* message type; both directions */
	atm_kptr_t vcc;		/* both directions */
	struct sockaddr_atmpvc addr; /* suggested value from kernel */
	struct atm_qos	qos;	/* both directions */
	int result;		/* to kernel only */
} __ATM_API_ALIGN;


#define SIOCSIFATMTCP	_IO('a',ATMIOC_ITF)	/* set ATMTCP mode */
#define ATMTCP_CREATE	_IO('a',ATMIOC_ITF+14)	/* create persistent ATMTCP
						   interface */
#define ATMTCP_REMOVE	_IO('a',ATMIOC_ITF+15)	/* destroy persistent ATMTCP
						   interface */


#ifdef __KERNEL__

struct atm_tcp_ops {
	int (*attach)(struct atm_vcc *vcc,int itf);
	int (*create_persistent)(int itf);
	int (*remove_persistent)(int itf);
	struct module *owner;
};

extern struct atm_tcp_ops atm_tcp_ops;

#endif

#endif
