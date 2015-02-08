

void set_main_if_addr(uint8_t *addr);
int main_if_was_up(void);
void interface_setup(struct net_device *dev);
int interface_open(struct net_device *dev);
int interface_release(struct net_device *dev);
struct net_device_stats *interface_stats(struct net_device *dev);
int interface_set_mac_addr(struct net_device *dev, void *addr);
int interface_change_mtu(struct net_device *dev, int new_mtu);
int interface_tx(struct sk_buff *skb, struct net_device *dev);
void interface_rx(struct sk_buff *skb, int hdr_size);
int my_skb_push(struct sk_buff *skb, unsigned int len);

extern unsigned char mainIfAddr[];
