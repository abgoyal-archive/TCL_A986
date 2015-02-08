

#ifndef _TIPC_BEARER_H
#define _TIPC_BEARER_H

#include "core.h"
#include "bcast.h"

#define MAX_BEARERS 8
#define MAX_MEDIA 4



struct media {
	int (*send_msg)(struct sk_buff *buf,
			struct tipc_bearer *b_ptr,
			struct tipc_media_addr *dest);
	int (*enable_bearer)(struct tipc_bearer *b_ptr);
	void (*disable_bearer)(struct tipc_bearer *b_ptr);
	char *(*addr2str)(struct tipc_media_addr *a,
			  char *str_buf, int str_size);
	struct tipc_media_addr bcast_addr;
	int bcast;
	u32 priority;
	u32 tolerance;
	u32 window;
	u32 type_id;
	char name[TIPC_MAX_MEDIA_NAME];
};


struct bearer {
	struct tipc_bearer publ;
	struct media *media;
	u32 priority;
	u32 detect_scope;
	u32 identity;
	struct link_req *link_req;
	struct list_head links;
	struct list_head cong_links;
	u32 continue_count;
	int active;
	char net_plane;
	struct tipc_node_map nodes;
};

struct bearer_name {
	char media_name[TIPC_MAX_MEDIA_NAME];
	char if_name[TIPC_MAX_IF_NAME];
};

struct link;

extern struct bearer tipc_bearers[];

void tipc_media_addr_printf(struct print_buf *pb, struct tipc_media_addr *a);
struct sk_buff *tipc_media_get_names(void);

struct sk_buff *tipc_bearer_get_names(void);
void tipc_bearer_add_dest(struct bearer *b_ptr, u32 dest);
void tipc_bearer_remove_dest(struct bearer *b_ptr, u32 dest);
void tipc_bearer_schedule(struct bearer *b_ptr, struct link *l_ptr);
struct bearer *tipc_bearer_find_interface(const char *if_name);
int tipc_bearer_resolve_congestion(struct bearer *b_ptr, struct link *l_ptr);
int tipc_bearer_congested(struct bearer *b_ptr, struct link *l_ptr);
int tipc_bearer_init(void);
void tipc_bearer_stop(void);
void tipc_bearer_lock_push(struct bearer *b_ptr);



static inline int tipc_bearer_send(struct bearer *b_ptr, struct sk_buff *buf,
				   struct tipc_media_addr *dest)
{
	return !b_ptr->media->send_msg(buf, &b_ptr->publ, dest);
}

#endif	/* _TIPC_BEARER_H */
