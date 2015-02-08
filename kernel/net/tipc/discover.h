

#ifndef _TIPC_DISCOVER_H
#define _TIPC_DISCOVER_H

#include "core.h"

struct link_req;

struct link_req *tipc_disc_init_link_req(struct bearer *b_ptr,
					 const struct tipc_media_addr *dest,
					 u32 dest_domain,
					 u32 req_links);
void tipc_disc_update_link_req(struct link_req *req);
void tipc_disc_stop_link_req(struct link_req *req);

void tipc_disc_recv_msg(struct sk_buff *buf, struct bearer *b_ptr);

void tipc_disc_link_event(u32 addr, char *name, int up);
#if 0
int  disc_create_link(const struct tipc_link_create *argv);
#endif

#endif
