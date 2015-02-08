

#ifndef CAIF_DEV_H_
#define CAIF_DEV_H_

#include <net/caif/caif_layer.h>
#include <net/caif/cfcnfg.h>
#include <linux/caif/caif_socket.h>
#include <linux/if.h>

struct caif_param {
	u16  size;
	u8   data[256];
};

struct caif_connect_request {
	enum caif_protocol_type protocol;
	struct sockaddr_caif sockaddr;
	enum caif_channel_priority priority;
	enum caif_link_selector link_selector;
	char link_name[16];
	struct caif_param param;
};

int caif_connect_client(struct caif_connect_request *config,
			   struct cflayer *client_layer);

int caif_disconnect_client(struct cflayer *client_layer);

void caif_release_client(struct cflayer *client_layer);

int connect_req_to_link_param(struct cfcnfg *cnfg,
				struct caif_connect_request *con_req,
				struct cfctrl_link_param *channel_setup_param);

struct cfcnfg *get_caif_conf(void);


#endif /* CAIF_DEV_H_ */
