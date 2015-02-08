

#ifndef __XEN_PUBLIC_EVENT_CHANNEL_H__
#define __XEN_PUBLIC_EVENT_CHANNEL_H__

#include <xen/interface/xen.h>

typedef uint32_t evtchn_port_t;
DEFINE_GUEST_HANDLE(evtchn_port_t);

#define EVTCHNOP_alloc_unbound	  6
struct evtchn_alloc_unbound {
	/* IN parameters */
	domid_t dom, remote_dom;
	/* OUT parameters */
	evtchn_port_t port;
};

#define EVTCHNOP_bind_interdomain 0
struct evtchn_bind_interdomain {
	/* IN parameters. */
	domid_t remote_dom;
	evtchn_port_t remote_port;
	/* OUT parameters. */
	evtchn_port_t local_port;
};

#define EVTCHNOP_bind_virq	  1
struct evtchn_bind_virq {
	/* IN parameters. */
	uint32_t virq;
	uint32_t vcpu;
	/* OUT parameters. */
	evtchn_port_t port;
};

#define EVTCHNOP_bind_pirq	  2
struct evtchn_bind_pirq {
	/* IN parameters. */
	uint32_t pirq;
#define BIND_PIRQ__WILL_SHARE 1
	uint32_t flags; /* BIND_PIRQ__* */
	/* OUT parameters. */
	evtchn_port_t port;
};

#define EVTCHNOP_bind_ipi	  7
struct evtchn_bind_ipi {
	uint32_t vcpu;
	/* OUT parameters. */
	evtchn_port_t port;
};

#define EVTCHNOP_close		  3
struct evtchn_close {
	/* IN parameters. */
	evtchn_port_t port;
};

#define EVTCHNOP_send		  4
struct evtchn_send {
	/* IN parameters. */
	evtchn_port_t port;
};

#define EVTCHNOP_status		  5
struct evtchn_status {
	/* IN parameters */
	domid_t  dom;
	evtchn_port_t port;
	/* OUT parameters */
#define EVTCHNSTAT_closed	0  /* Channel is not in use.		     */
#define EVTCHNSTAT_unbound	1  /* Channel is waiting interdom connection.*/
#define EVTCHNSTAT_interdomain	2  /* Channel is connected to remote domain. */
#define EVTCHNSTAT_pirq		3  /* Channel is bound to a phys IRQ line.   */
#define EVTCHNSTAT_virq		4  /* Channel is bound to a virtual IRQ line */
#define EVTCHNSTAT_ipi		5  /* Channel is bound to a virtual IPI line */
	uint32_t status;
	uint32_t vcpu;		   /* VCPU to which this channel is bound.   */
	union {
		struct {
			domid_t dom;
		} unbound; /* EVTCHNSTAT_unbound */
		struct {
			domid_t dom;
			evtchn_port_t port;
		} interdomain; /* EVTCHNSTAT_interdomain */
		uint32_t pirq;	    /* EVTCHNSTAT_pirq	      */
		uint32_t virq;	    /* EVTCHNSTAT_virq	      */
	} u;
};

#define EVTCHNOP_bind_vcpu	  8
struct evtchn_bind_vcpu {
	/* IN parameters. */
	evtchn_port_t port;
	uint32_t vcpu;
};

#define EVTCHNOP_unmask		  9
struct evtchn_unmask {
	/* IN parameters. */
	evtchn_port_t port;
};

struct evtchn_op {
	uint32_t cmd; /* EVTCHNOP_* */
	union {
		struct evtchn_alloc_unbound    alloc_unbound;
		struct evtchn_bind_interdomain bind_interdomain;
		struct evtchn_bind_virq	       bind_virq;
		struct evtchn_bind_pirq	       bind_pirq;
		struct evtchn_bind_ipi	       bind_ipi;
		struct evtchn_close	       close;
		struct evtchn_send	       send;
		struct evtchn_status	       status;
		struct evtchn_bind_vcpu	       bind_vcpu;
		struct evtchn_unmask	       unmask;
	} u;
};
DEFINE_GUEST_HANDLE_STRUCT(evtchn_op);

#endif /* __XEN_PUBLIC_EVENT_CHANNEL_H__ */
