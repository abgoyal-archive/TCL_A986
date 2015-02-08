

#ifndef _FCOE_H_
#define _FCOE_H_

#include <linux/skbuff.h>
#include <linux/kthread.h>

#define FCOE_MAX_QUEUE_DEPTH	256
#define FCOE_LOW_QUEUE_DEPTH	32

#define FCOE_WORD_TO_BYTE	4

#define FCOE_VERSION	"0.1"
#define FCOE_NAME	"fcoe"
#define FCOE_VENDOR	"Open-FCoE.org"

#define FCOE_MAX_LUN		0xFFFF
#define FCOE_MAX_FCP_TARGET	256

#define FCOE_MAX_OUTSTANDING_COMMANDS	1024

#define FCOE_MIN_XID		0x0000	/* the min xid supported by fcoe_sw */
#define FCOE_MAX_XID		0x0FFF	/* the max xid supported by fcoe_sw */

#define FCOE_MTU	2158

unsigned int fcoe_debug_logging;
module_param_named(debug_logging, fcoe_debug_logging, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(debug_logging, "a bit mask of logging levels");

#define FCOE_LOGGING	    0x01 /* General logging, not categorized */
#define FCOE_NETDEV_LOGGING 0x02 /* Netdevice logging */

#define FCOE_CHECK_LOGGING(LEVEL, CMD)					\
do {                                                            	\
	if (unlikely(fcoe_debug_logging & LEVEL))			\
		do {							\
			CMD;						\
		} while (0);						\
} while (0)

#define FCOE_DBG(fmt, args...)						\
	FCOE_CHECK_LOGGING(FCOE_LOGGING,				\
			   printk(KERN_INFO "fcoe: " fmt, ##args);)

#define FCOE_NETDEV_DBG(netdev, fmt, args...)			\
	FCOE_CHECK_LOGGING(FCOE_NETDEV_LOGGING,			\
			   printk(KERN_INFO "fcoe: %s: " fmt,	\
				  netdev->name, ##args);)

struct fcoe_percpu_s {
	struct task_struct *thread;
	struct sk_buff_head fcoe_rx_list;
	struct page *crc_eof_page;
	int crc_eof_offset;
};

struct fcoe_interface {
	struct list_head   list;
	struct net_device  *netdev;
	struct packet_type fcoe_packet_type;
	struct packet_type fip_packet_type;
	struct fcoe_ctlr   ctlr;
	struct fc_exch_mgr *oem;
	struct kref	   kref;
};

struct fcoe_port {
	struct fcoe_interface *fcoe;
	struct fc_lport	      *lport;
	struct sk_buff_head   fcoe_pending_queue;
	u8		      fcoe_pending_queue_active;
	struct timer_list     timer;
	struct work_struct    destroy_work;
	u8		      data_src_addr[ETH_ALEN];
};

#define fcoe_from_ctlr(fip) container_of(fip, struct fcoe_interface, ctlr)

static inline struct net_device *fcoe_netdev(const struct fc_lport *lport)
{
	return ((struct fcoe_port *)lport_priv(lport))->fcoe->netdev;
}

#endif /* _FCOE_H_ */
