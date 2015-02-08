
#ifndef _CHELSIO_L2T_H
#define _CHELSIO_L2T_H

#include <linux/spinlock.h>
#include "t3cdev.h"
#include <asm/atomic.h>

enum {
	L2T_STATE_VALID,	/* entry is up to date */
	L2T_STATE_STALE,	/* entry may be used but needs revalidation */
	L2T_STATE_RESOLVING,	/* entry needs address resolution */
	L2T_STATE_UNUSED	/* entry not in use */
};

struct neighbour;
struct sk_buff;

struct l2t_entry {
	u16 state;		/* entry state */
	u16 idx;		/* entry index */
	u32 addr;		/* dest IP address */
	int ifindex;		/* neighbor's net_device's ifindex */
	u16 smt_idx;		/* SMT index */
	u16 vlan;		/* VLAN TCI (id: bits 0-11, prio: 13-15 */
	struct neighbour *neigh;	/* associated neighbour */
	struct l2t_entry *first;	/* start of hash chain */
	struct l2t_entry *next;	/* next l2t_entry on chain */
	struct sk_buff_head arpq;	/* queue of packets awaiting resolution */
	spinlock_t lock;
	atomic_t refcnt;	/* entry reference count */
	u8 dmac[6];		/* neighbour's MAC address */
};

struct l2t_data {
	unsigned int nentries;	/* number of entries */
	struct l2t_entry *rover;	/* starting point for next allocation */
	atomic_t nfree;		/* number of free entries */
	rwlock_t lock;
	struct l2t_entry l2tab[0];
};

typedef void (*arp_failure_handler_func)(struct t3cdev * dev,
					 struct sk_buff * skb);

struct l2t_skb_cb {
	arp_failure_handler_func arp_failure_handler;
};

#define L2T_SKB_CB(skb) ((struct l2t_skb_cb *)(skb)->cb)

static inline void set_arp_failure_handler(struct sk_buff *skb,
					   arp_failure_handler_func hnd)
{
	L2T_SKB_CB(skb)->arp_failure_handler = hnd;
}

#define L2DATA(dev) ((dev)->l2opt)

#define W_TCB_L2T_IX    0
#define S_TCB_L2T_IX    7
#define M_TCB_L2T_IX    0x7ffULL
#define V_TCB_L2T_IX(x) ((x) << S_TCB_L2T_IX)

void t3_l2e_free(struct l2t_data *d, struct l2t_entry *e);
void t3_l2t_update(struct t3cdev *dev, struct neighbour *neigh);
struct l2t_entry *t3_l2t_get(struct t3cdev *cdev, struct neighbour *neigh,
			     struct net_device *dev);
int t3_l2t_send_slow(struct t3cdev *dev, struct sk_buff *skb,
		     struct l2t_entry *e);
void t3_l2t_send_event(struct t3cdev *dev, struct l2t_entry *e);
struct l2t_data *t3_init_l2t(unsigned int l2t_capacity);
void t3_free_l2t(struct l2t_data *d);

int cxgb3_ofld_send(struct t3cdev *dev, struct sk_buff *skb);

static inline int l2t_send(struct t3cdev *dev, struct sk_buff *skb,
			   struct l2t_entry *e)
{
	if (likely(e->state == L2T_STATE_VALID))
		return cxgb3_ofld_send(dev, skb);
	return t3_l2t_send_slow(dev, skb, e);
}

static inline void l2t_release(struct l2t_data *d, struct l2t_entry *e)
{
	if (atomic_dec_and_test(&e->refcnt))
		t3_l2e_free(d, e);
}

static inline void l2t_hold(struct l2t_data *d, struct l2t_entry *e)
{
	if (atomic_add_return(1, &e->refcnt) == 1)	/* 0 -> 1 transition */
		atomic_dec(&d->nfree);
}

#endif
