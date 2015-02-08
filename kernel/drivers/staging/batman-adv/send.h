

#include "types.h"

void send_own_packet_work(struct work_struct *work);
int send_skb_packet(struct sk_buff *skb,
				struct batman_if *batman_if,
				uint8_t *dst_addr);
void send_raw_packet(unsigned char *pack_buff, int pack_buff_len,
		     struct batman_if *batman_if, uint8_t *dst_addr);
void schedule_own_packet(struct batman_if *batman_if);
void schedule_forward_packet(struct orig_node *orig_node,
			     struct ethhdr *ethhdr,
			     struct batman_packet *batman_packet,
			     uint8_t directlink, int hna_buff_len,
			     struct batman_if *if_outgoing);
int  add_bcast_packet_to_list(struct sk_buff *skb);
void send_outstanding_bcast_packet(struct work_struct *work);
void send_outstanding_bat_packet(struct work_struct *work);
void purge_outstanding_packets(struct batman_if *batman_if);
