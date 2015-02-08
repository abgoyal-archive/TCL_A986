





#ifndef TYPES_H
#define TYPES_H

#include "packet.h"
#include "bitarray.h"

#define BAT_HEADER_LEN (sizeof(struct ethhdr) + \
	((sizeof(struct unicast_packet) > sizeof(struct bcast_packet) ? \
	 sizeof(struct unicast_packet) : \
	 sizeof(struct bcast_packet))))


struct batman_if {
	struct list_head list;
	int16_t if_num;
	char *dev;
	char if_status;
	char addr_str[ETH_STR_LEN];
	struct net_device *net_dev;
	atomic_t seqno;
	unsigned char *packet_buff;
	int packet_len;
	struct kobject *hardif_obj;
	struct rcu_head rcu;

};

struct orig_node {
	uint8_t orig[ETH_ALEN];
	struct neigh_node *router;
	TYPE_OF_WORD *bcast_own;
	uint8_t *bcast_own_sum;
	uint8_t tq_own;
	int tq_asym_penalty;
	unsigned long last_valid;
	unsigned long bcast_seqno_reset;
	unsigned long batman_seqno_reset;
	uint8_t  flags;
	unsigned char *hna_buff;
	int16_t  hna_buff_len;
	uint16_t last_real_seqno;
	uint8_t last_ttl;
	TYPE_OF_WORD bcast_bits[NUM_WORDS];
	uint16_t last_bcast_seqno;
	struct list_head neigh_list;
};

struct neigh_node {
	struct list_head list;
	uint8_t addr[ETH_ALEN];
	uint8_t real_packet_count;
	uint8_t tq_recv[TQ_GLOBAL_WINDOW_SIZE];
	uint8_t tq_index;
	uint8_t tq_avg;
	uint8_t last_ttl;
	unsigned long last_valid;
	TYPE_OF_WORD real_bits[NUM_WORDS];
	struct orig_node *orig_node;
	struct batman_if *if_incoming;
};

struct bat_priv {
	struct net_device_stats stats;
	atomic_t aggregation_enabled;
	atomic_t vis_mode;
	atomic_t orig_interval;
	char num_ifaces;
	struct batman_if *primary_if;
	struct kobject *mesh_obj;
};

struct device_client {
	struct list_head queue_list;
	unsigned int queue_len;
	unsigned char index;
	spinlock_t lock;
	wait_queue_head_t queue_wait;
};

struct device_packet {
	struct list_head list;
	struct icmp_packet icmp_packet;
};

struct hna_local_entry {
	uint8_t addr[ETH_ALEN];
	unsigned long last_seen;
	char never_purge;
};

struct hna_global_entry {
	uint8_t addr[ETH_ALEN];
	struct orig_node *orig_node;
};

struct forw_packet {
	struct hlist_node list;
	unsigned long send_time;
	uint8_t own;
	struct sk_buff *skb;
	unsigned char *packet_buff;
	uint16_t packet_len;
	uint32_t direct_link_flags;
	uint8_t num_packets;
	struct delayed_work delayed_work;
	struct batman_if *if_incoming;
};

struct if_list_entry {
	uint8_t addr[ETH_ALEN];
	bool primary;
	struct hlist_node list;
};

#endif
