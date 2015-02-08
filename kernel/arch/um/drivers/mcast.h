

#ifndef __DRIVERS_MCAST_H
#define __DRIVERS_MCAST_H

#include "net_user.h"

struct mcast_data {
	char *addr;
	unsigned short port;
	void *mcast_addr;
	int ttl;
	void *dev;
};

extern const struct net_user_info mcast_user_info;

extern int mcast_user_write(int fd, void *buf, int len, 
			    struct mcast_data *pri);

#endif
