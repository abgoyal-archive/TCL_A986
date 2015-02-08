

#ifndef __CXGB4_L2T_H
#define __CXGB4_L2T_H

#include <linux/spinlock.h>
#include <linux/if_ether.h>
#include <asm/atomic.h>

struct adapter;
struct l2t_data;
struct neighbour;
struct net_device;
struct file_operations;
struct cpl_l2t_write_rpl;

struct l2t_entry {
	u16 state;                  /* entry state */
	u16 idx;                    /* entry index */
	u32 addr[4];                /* next hop IP or IPv6 address */
	int ifindex;                /* neighbor's net_device's ifindex */
	struct neighbour *neigh;    /* associated neighbour */
	struct l2t_entry *first;    /* start of hash chain */
	struct l2t_entry *next;     /* next l2t_entry on chain */
	struct sk_buff *arpq_head;  /* queue of packets awaiting resolution */
	struct sk_buff *arpq_tail;
	spinlock_t lock;
	atomic_t refcnt;            /* entry reference count */
	u16 hash;                   /* hash bucket the entry is on */
	u16 vlan;                   /* VLAN TCI (id: bits 0-11, prio: 13-15 */
	u8 v6;                      /* whether entry is for IPv6 */
	u8 lport;                   /* associated offload logical interface */
	u8 dmac[ETH_ALEN];          /* neighbour's MAC address */
};

typedef void (*arp_err_handler_t)(void *handle, struct sk_buff *skb);

struct l2t_skb_cb {
	void *handle;
	arp_err_handler_t arp_err_handler;
};

#define L2T_SKB_CB(skb) ((struct l2t_skb_cb *)(skb)->cb)

static inline void t4_set_arp_err_handler(struct sk_buff *skb, void *handle,
					  arp_err_handler_t handler)
{
	L2T_SKB_CB(skb)->handle = handle;
	L2T_SKB_CB(skb)->arp_err_handler = handler;
}

void cxgb4_l2t_release(struct l2t_entry *e);
int cxgb4_l2t_send(struct net_device *dev, struct sk_buff *skb,
		   struct l2t_entry *e);
struct l2t_entry *cxgb4_l2t_get(struct l2t_data *d, struct neighbour *neigh,
				const struct net_device *physdev,
				unsigned int priority);

void t4_l2t_update(struct adapter *adap, struct neighbour *neigh);
struct l2t_entry *t4_l2t_alloc_switching(struct l2t_data *d);
int t4_l2t_set_switching(struct adapter *adap, struct l2t_entry *e, u16 vlan,
			 u8 port, u8 *eth_addr);
struct l2t_data *t4_init_l2t(void);
void do_l2t_write_rpl(struct adapter *p, const struct cpl_l2t_write_rpl *rpl);

extern const struct file_operations t4_l2t_fops;
#endif  /* __CXGB4_L2T_H */
