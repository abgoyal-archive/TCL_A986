

#include "main.h"

/* is there another aggregated packet here? */
static inline int aggregated_packet(int buff_pos, int packet_len, int num_hna)
{
	int next_buff_pos = buff_pos + BAT_PACKET_LEN + (num_hna * ETH_ALEN);

	return (next_buff_pos <= packet_len) &&
		(next_buff_pos <= MAX_AGGREGATION_BYTES);
}

void add_bat_packet_to_list(struct bat_priv *bat_priv,
			    unsigned char *packet_buff, int packet_len,
			    struct batman_if *if_incoming, char own_packet,
			    unsigned long send_time);
void receive_aggr_bat_packet(struct ethhdr *ethhdr, unsigned char *packet_buff,
			     int packet_len, struct batman_if *if_incoming);
