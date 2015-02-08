

#ifndef _TIPC_CORE_H
#define _TIPC_CORE_H

#include <linux/tipc.h>
#include <linux/tipc_config.h>
#include <net/tipc/tipc_msg.h>
#include <net/tipc/tipc_port.h>
#include <net/tipc/tipc_bearer.h>
#include <net/tipc/tipc.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <asm/atomic.h>
#include <asm/hardirq.h>
#include <linux/netdevice.h>
#include <linux/in.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>


#define TIPC_MOD_VER "2.0.0"


#define assert(i)  BUG_ON(!(i))



extern struct print_buf *const TIPC_NULL;
extern struct print_buf *const TIPC_CONS;
extern struct print_buf *const TIPC_LOG;

void tipc_printf(struct print_buf *, const char *fmt, ...);


#ifndef TIPC_OUTPUT
#define TIPC_OUTPUT TIPC_LOG
#endif


#ifdef CONFIG_TIPC_DEBUG

#define err(fmt, arg...)  tipc_printf(TIPC_OUTPUT, \
					KERN_ERR "TIPC: " fmt, ## arg)
#define warn(fmt, arg...) tipc_printf(TIPC_OUTPUT, \
					KERN_WARNING "TIPC: " fmt, ## arg)
#define info(fmt, arg...) tipc_printf(TIPC_OUTPUT, \
					KERN_NOTICE "TIPC: " fmt, ## arg)

#else

#define err(fmt, arg...)  printk(KERN_ERR "TIPC: " fmt , ## arg)
#define info(fmt, arg...) printk(KERN_INFO "TIPC: " fmt , ## arg)
#define warn(fmt, arg...) printk(KERN_WARNING "TIPC: " fmt , ## arg)

#endif


#ifndef DBG_OUTPUT
#define DBG_OUTPUT TIPC_NULL
#endif


#ifdef CONFIG_TIPC_DEBUG

#define dbg(fmt, arg...)  \
	do { \
		if (DBG_OUTPUT != TIPC_NULL) \
			tipc_printf(DBG_OUTPUT, fmt, ## arg); \
	} while (0)
#define msg_dbg(msg, txt) \
	do { \
		if (DBG_OUTPUT != TIPC_NULL) \
			tipc_msg_dbg(DBG_OUTPUT, msg, txt); \
	} while (0)
#define dump(fmt, arg...) \
	do { \
		if (DBG_OUTPUT != TIPC_NULL) \
			tipc_dump_dbg(DBG_OUTPUT, fmt, ##arg); \
	} while (0)

void tipc_msg_dbg(struct print_buf *, struct tipc_msg *, const char *);
void tipc_dump_dbg(struct print_buf *, const char *fmt, ...);

#else

#define dbg(fmt, arg...)	do {} while (0)
#define msg_dbg(msg, txt)	do {} while (0)
#define dump(fmt, arg...)	do {} while (0)

#define tipc_msg_dbg(...)	do {} while (0)
#define tipc_dump_dbg(...)	do {} while (0)

#endif



#define ELINKCONG EAGAIN	/* link congestion <=> resource unavailable */


extern u32 tipc_own_addr;
extern int tipc_max_zones;
extern int tipc_max_clusters;
extern int tipc_max_nodes;
extern int tipc_max_slaves;
extern int tipc_max_ports;
extern int tipc_max_subscriptions;
extern int tipc_max_publications;
extern int tipc_net_id;
extern int tipc_remote_management;


extern int tipc_mode;
extern int tipc_random;
extern const char tipc_alphabet[];
extern atomic_t tipc_user_count;



extern int  tipc_core_start(void);
extern void tipc_core_stop(void);
extern int  tipc_core_start_net(unsigned long addr);
extern void tipc_core_stop_net(void);
extern int  tipc_handler_start(void);
extern void tipc_handler_stop(void);
extern int  tipc_netlink_start(void);
extern void tipc_netlink_stop(void);
extern int  tipc_socket_init(void);
extern void tipc_socket_stop(void);

static inline int delimit(int val, int min, int max)
{
	if (val > max)
		return max;
	if (val < min)
		return min;
	return val;
}



typedef void (*Handler) (unsigned long);

u32 tipc_k_signal(Handler routine, unsigned long argument);


static inline void k_init_timer(struct timer_list *timer, Handler routine,
				unsigned long argument)
{
	dbg("initializing timer %p\n", timer);
	setup_timer(timer, routine, argument);
}


static inline void k_start_timer(struct timer_list *timer, unsigned long msec)
{
	dbg("starting timer %p for %u\n", timer, msec);
	mod_timer(timer, jiffies + msecs_to_jiffies(msec) + 1);
}


static inline void k_cancel_timer(struct timer_list *timer)
{
	dbg("cancelling timer %p\n", timer);
	del_timer_sync(timer);
}


static inline void k_term_timer(struct timer_list *timer)
{
	dbg("terminating timer %p\n", timer);
}



#define BUF_HEADROOM LL_MAX_HEADER

struct tipc_skb_cb {
	void *handle;
};

#define TIPC_SKB_CB(__skb) ((struct tipc_skb_cb *)&((__skb)->cb[0]))


static inline struct tipc_msg *buf_msg(struct sk_buff *skb)
{
	return (struct tipc_msg *)skb->data;
}

extern struct sk_buff *buf_acquire(u32 size);


static inline void buf_discard(struct sk_buff *skb)
{
	kfree_skb(skb);
}


static inline int buf_linearize(struct sk_buff *skb)
{
	return skb_linearize(skb);
}

#endif
