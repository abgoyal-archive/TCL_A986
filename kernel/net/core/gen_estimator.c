

#include <asm/uaccess.h>
#include <asm/system.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/socket.h>
#include <linux/sockios.h>
#include <linux/in.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/rtnetlink.h>
#include <linux/init.h>
#include <linux/rbtree.h>
#include <linux/slab.h>
#include <net/sock.h>
#include <net/gen_stats.h>


#define EST_MAX_INTERVAL	5

struct gen_estimator
{
	struct list_head	list;
	struct gnet_stats_basic_packed	*bstats;
	struct gnet_stats_rate_est	*rate_est;
	spinlock_t		*stats_lock;
	int			ewma_log;
	u64			last_bytes;
	u64			avbps;
	u32			last_packets;
	u32			avpps;
	struct rcu_head		e_rcu;
	struct rb_node		node;
};

struct gen_estimator_head
{
	struct timer_list	timer;
	struct list_head	list;
};

static struct gen_estimator_head elist[EST_MAX_INTERVAL+1];

/* Protects against NULL dereference */
static DEFINE_RWLOCK(est_lock);

/* Protects against soft lockup during large deletion */
static struct rb_root est_root = RB_ROOT;
static DEFINE_SPINLOCK(est_tree_lock);

static void est_timer(unsigned long arg)
{
	int idx = (int)arg;
	struct gen_estimator *e;

	rcu_read_lock();
	list_for_each_entry_rcu(e, &elist[idx].list, list) {
		u64 nbytes;
		u64 brate;
		u32 npackets;
		u32 rate;

		spin_lock(e->stats_lock);
		read_lock(&est_lock);
		if (e->bstats == NULL)
			goto skip;

		nbytes = e->bstats->bytes;
		npackets = e->bstats->packets;
		brate = (nbytes - e->last_bytes)<<(7 - idx);
		e->last_bytes = nbytes;
		e->avbps += (brate >> e->ewma_log) - (e->avbps >> e->ewma_log);
		e->rate_est->bps = (e->avbps+0xF)>>5;

		rate = (npackets - e->last_packets)<<(12 - idx);
		e->last_packets = npackets;
		e->avpps += (rate >> e->ewma_log) - (e->avpps >> e->ewma_log);
		e->rate_est->pps = (e->avpps+0x1FF)>>10;
skip:
		read_unlock(&est_lock);
		spin_unlock(e->stats_lock);
	}

	if (!list_empty(&elist[idx].list))
		mod_timer(&elist[idx].timer, jiffies + ((HZ/4) << idx));
	rcu_read_unlock();
}

static void gen_add_node(struct gen_estimator *est)
{
	struct rb_node **p = &est_root.rb_node, *parent = NULL;

	while (*p) {
		struct gen_estimator *e;

		parent = *p;
		e = rb_entry(parent, struct gen_estimator, node);

		if (est->bstats > e->bstats)
			p = &parent->rb_right;
		else
			p = &parent->rb_left;
	}
	rb_link_node(&est->node, parent, p);
	rb_insert_color(&est->node, &est_root);
}

static
struct gen_estimator *gen_find_node(const struct gnet_stats_basic_packed *bstats,
				    const struct gnet_stats_rate_est *rate_est)
{
	struct rb_node *p = est_root.rb_node;

	while (p) {
		struct gen_estimator *e;

		e = rb_entry(p, struct gen_estimator, node);

		if (bstats > e->bstats)
			p = p->rb_right;
		else if (bstats < e->bstats || rate_est != e->rate_est)
			p = p->rb_left;
		else
			return e;
	}
	return NULL;
}

int gen_new_estimator(struct gnet_stats_basic_packed *bstats,
		      struct gnet_stats_rate_est *rate_est,
		      spinlock_t *stats_lock,
		      struct nlattr *opt)
{
	struct gen_estimator *est;
	struct gnet_estimator *parm = nla_data(opt);
	int idx;

	if (nla_len(opt) < sizeof(*parm))
		return -EINVAL;

	if (parm->interval < -2 || parm->interval > 3)
		return -EINVAL;

	est = kzalloc(sizeof(*est), GFP_KERNEL);
	if (est == NULL)
		return -ENOBUFS;

	idx = parm->interval + 2;
	est->bstats = bstats;
	est->rate_est = rate_est;
	est->stats_lock = stats_lock;
	est->ewma_log = parm->ewma_log;
	est->last_bytes = bstats->bytes;
	est->avbps = rate_est->bps<<5;
	est->last_packets = bstats->packets;
	est->avpps = rate_est->pps<<10;

	spin_lock(&est_tree_lock);
	if (!elist[idx].timer.function) {
		INIT_LIST_HEAD(&elist[idx].list);
		setup_timer(&elist[idx].timer, est_timer, idx);
	}

	if (list_empty(&elist[idx].list))
		mod_timer(&elist[idx].timer, jiffies + ((HZ/4) << idx));

	list_add_rcu(&est->list, &elist[idx].list);
	gen_add_node(est);
	spin_unlock(&est_tree_lock);

	return 0;
}
EXPORT_SYMBOL(gen_new_estimator);

static void __gen_kill_estimator(struct rcu_head *head)
{
	struct gen_estimator *e = container_of(head,
					struct gen_estimator, e_rcu);
	kfree(e);
}

void gen_kill_estimator(struct gnet_stats_basic_packed *bstats,
			struct gnet_stats_rate_est *rate_est)
{
	struct gen_estimator *e;

	spin_lock(&est_tree_lock);
	while ((e = gen_find_node(bstats, rate_est))) {
		rb_erase(&e->node, &est_root);

		write_lock_bh(&est_lock);
		e->bstats = NULL;
		write_unlock_bh(&est_lock);

		list_del_rcu(&e->list);
		call_rcu(&e->e_rcu, __gen_kill_estimator);
	}
	spin_unlock(&est_tree_lock);
}
EXPORT_SYMBOL(gen_kill_estimator);

int gen_replace_estimator(struct gnet_stats_basic_packed *bstats,
			  struct gnet_stats_rate_est *rate_est,
			  spinlock_t *stats_lock, struct nlattr *opt)
{
	gen_kill_estimator(bstats, rate_est);
	return gen_new_estimator(bstats, rate_est, stats_lock, opt);
}
EXPORT_SYMBOL(gen_replace_estimator);

bool gen_estimator_active(const struct gnet_stats_basic_packed *bstats,
			  const struct gnet_stats_rate_est *rate_est)
{
	bool res;

	ASSERT_RTNL();

	spin_lock(&est_tree_lock);
	res = gen_find_node(bstats, rate_est) != NULL;
	spin_unlock(&est_tree_lock);

	return res;
}
EXPORT_SYMBOL(gen_estimator_active);
