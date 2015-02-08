
#ifndef _PPP_CHANNEL_H_
#define _PPP_CHANNEL_H_

#include <linux/list.h>
#include <linux/skbuff.h>
#include <linux/poll.h>
#include <net/net_namespace.h>

struct ppp_channel;

struct ppp_channel_ops {
	/* Send a packet (or multilink fragment) on this channel.
	   Returns 1 if it was accepted, 0 if not. */
	int	(*start_xmit)(struct ppp_channel *, struct sk_buff *);
	/* Handle an ioctl call that has come in via /dev/ppp. */
	int	(*ioctl)(struct ppp_channel *, unsigned int, unsigned long);
};

struct ppp_channel {
	void		*private;	/* channel private data */
	struct ppp_channel_ops *ops;	/* operations for this channel */
	int		mtu;		/* max transmit packet size */
	int		hdrlen;		/* amount of headroom channel needs */
	void		*ppp;		/* opaque to channel */
	int		speed;		/* transfer rate (bytes/second) */
	/* the following is not used at present */
	int		latency;	/* overhead time in milliseconds */
};

#ifdef __KERNEL__
/* Called by the channel when it can send some more data. */
extern void ppp_output_wakeup(struct ppp_channel *);

extern void ppp_input(struct ppp_channel *, struct sk_buff *);

extern void ppp_input_error(struct ppp_channel *, int code);

/* Attach a channel to a given PPP unit in specified net. */
extern int ppp_register_net_channel(struct net *, struct ppp_channel *);

/* Attach a channel to a given PPP unit. */
extern int ppp_register_channel(struct ppp_channel *);

/* Detach a channel from its PPP unit (e.g. on hangup). */
extern void ppp_unregister_channel(struct ppp_channel *);

/* Get the channel number for a channel */
extern int ppp_channel_index(struct ppp_channel *);

/* Get the unit number associated with a channel, or -1 if none */
extern int ppp_unit_number(struct ppp_channel *);

/* Get the device name associated with a channel, or NULL if none */
extern char *ppp_dev_name(struct ppp_channel *);


#endif /* __KERNEL__ */
#endif
