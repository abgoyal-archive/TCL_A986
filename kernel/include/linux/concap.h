

#ifndef _LINUX_CONCAP_H
#define _LINUX_CONCAP_H

#include <linux/skbuff.h>
#include <linux/netdevice.h>


struct concap_proto_ops;
struct concap_device_ops;

struct concap_proto{
	struct net_device *net_dev;	/* net device using our service  */
	struct concap_device_ops *dops;	/* callbacks provided by device */
 	struct concap_proto_ops  *pops;	/* callbacks provided by us */
 	spinlock_t lock;
	int flags;
	void *proto_data;		/* protocol specific private data, to
					   be accessed via *pops methods only*/
	/*
	  :
	  whatever 
	  :
	  */
};

struct concap_device_ops{

	/* to request data is submitted by device*/ 
	int (*data_req)(struct concap_proto *, struct sk_buff *);

	/* Control methods must be set to NULL by devices which do not
	   support connection control.*/
	/* to request a connection is set up */ 
	int (*connect_req)(struct concap_proto *);

	/* to request a connection is released */
	int (*disconn_req)(struct concap_proto *);	
};

struct concap_proto_ops{

	/* create a new encapsulation protocol instance of same type */
	struct concap_proto *  (*proto_new) (void);

	/* delete encapsulation protocol instance and free all its resources.
	   cprot may no loger be referenced after calling this */
	void (*proto_del)(struct concap_proto *cprot);

	/* initialize the protocol's data. To be called at interface startup
	   or when the device driver resets the interface. All services of the
	   encapsulation protocol may be used after this*/
	int (*restart)(struct concap_proto *cprot, 
		       struct net_device *ndev,
		       struct concap_device_ops *dops);

	/* inactivate an encapsulation protocol instance. The encapsulation
	   protocol may not call any *dops methods after this. */
	int (*close)(struct concap_proto *cprot);

	/* process a frame handed down to us by upper layer */
	int (*encap_and_xmit)(struct concap_proto *cprot, struct sk_buff *skb);

	/* to be called for each data entity received from lower layer*/ 
	int (*data_ind)(struct concap_proto *cprot, struct sk_buff *skb);

	/* to be called when a connection was set up/down.
	   Protocols that don't process these primitives might fill in
	   dummy methods here */
	int (*connect_ind)(struct concap_proto *cprot);
	int (*disconn_ind)(struct concap_proto *cprot);
  /*
    Some network device support functions, like net_header(), rebuild_header(),
    and others, that depend solely on the encapsulation protocol, might
    be provided here, too. The net device would just fill them in its
    corresponding fields when it is opened.
    */
};

extern int concap_nop(struct concap_proto *cprot); 

extern int concap_drop_skb(struct concap_proto *cprot, struct sk_buff *skb);
#endif
