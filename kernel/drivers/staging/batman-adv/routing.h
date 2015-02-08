

#include "types.h"

extern wait_queue_head_t thread_wait;

void slide_own_bcast_window(struct batman_if *batman_if);
void receive_bat_packet(struct ethhdr *ethhdr,
				struct batman_packet *batman_packet,
				unsigned char *hna_buff, int hna_buff_len,
				struct batman_if *if_incoming);
void update_routes(struct orig_node *orig_node,
				struct neigh_node *neigh_node,
				unsigned char *hna_buff, int hna_buff_len);
int recv_icmp_packet(struct sk_buff *skb);
int recv_unicast_packet(struct sk_buff *skb);
int recv_bcast_packet(struct sk_buff *skb);
int recv_vis_packet(struct sk_buff *skb);
int recv_bat_packet(struct sk_buff *skb,
				struct batman_if *batman_if);
