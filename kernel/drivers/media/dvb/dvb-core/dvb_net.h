

#ifndef _DVB_NET_H_
#define _DVB_NET_H_

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#include "dvbdev.h"

#define DVB_NET_DEVICES_MAX 10

struct dvb_net {
	struct dvb_device *dvbdev;
	struct net_device *device[DVB_NET_DEVICES_MAX];
	int state[DVB_NET_DEVICES_MAX];
	unsigned int exit:1;
	struct dmx_demux *demux;
};


void dvb_net_release(struct dvb_net *);
int  dvb_net_init(struct dvb_adapter *, struct dvb_net *, struct dmx_demux *);

#endif
