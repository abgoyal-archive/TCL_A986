

#define IF_NOT_IN_USE 0
#define IF_TO_BE_REMOVED 1
#define IF_INACTIVE 2
#define IF_ACTIVE 3
#define IF_TO_BE_ACTIVATED 4
#define IF_I_WANT_YOU 5

extern struct notifier_block hard_if_notifier;

struct batman_if *get_batman_if_by_netdev(struct net_device *net_dev);
int hardif_enable_interface(struct batman_if *batman_if);
void hardif_disable_interface(struct batman_if *batman_if);
void hardif_remove_interfaces(void);
int batman_skb_recv(struct sk_buff *skb,
				struct net_device *dev,
				struct packet_type *ptype,
				struct net_device *orig_dev);
int hardif_min_mtu(void);
void update_min_mtu(void);
