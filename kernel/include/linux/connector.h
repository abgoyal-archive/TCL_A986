

#ifndef __CONNECTOR_H
#define __CONNECTOR_H

#include <linux/types.h>

#define CN_IDX_PROC			0x1
#define CN_VAL_PROC			0x1
#define CN_IDX_CIFS			0x2
#define CN_VAL_CIFS                     0x1
#define CN_W1_IDX			0x3	/* w1 communication */
#define CN_W1_VAL			0x1
#define CN_IDX_V86D			0x4
#define CN_VAL_V86D_UVESAFB		0x1
#define CN_IDX_BB			0x5	/* BlackBoard, from the TSP GPL sampling framework */
#define CN_DST_IDX			0x6
#define CN_DST_VAL			0x1
#define CN_IDX_DM			0x7	/* Device Mapper */
#define CN_VAL_DM_USERSPACE_LOG		0x1
#define CN_IDX_DRBD			0x8
#define CN_VAL_DRBD			0x1

#define CN_NETLINK_USERS		8

#define CONNECTOR_MAX_MSG_SIZE		16384


struct cb_id {
	__u32 idx;
	__u32 val;
};

struct cn_msg {
	struct cb_id id;

	__u32 seq;
	__u32 ack;

	__u16 len;		/* Length of the following data */
	__u16 flags;
	__u8 data[0];
};

#ifdef __KERNEL__

#include <asm/atomic.h>

#include <linux/list.h>
#include <linux/workqueue.h>

#include <net/sock.h>

#define CN_CBQ_NAMELEN		32

struct cn_queue_dev {
	atomic_t refcnt;
	unsigned char name[CN_CBQ_NAMELEN];

	struct workqueue_struct *cn_queue;
	/* Sent to kevent to create cn_queue only when needed */
	struct work_struct wq_creation;
	/* Tell if the wq_creation job is pending/completed */
	atomic_t wq_requested;
	/* Wait for cn_queue to be created */
	wait_queue_head_t wq_created;

	struct list_head queue_list;
	spinlock_t queue_lock;

	struct sock *nls;
};

struct cn_callback_id {
	unsigned char name[CN_CBQ_NAMELEN];
	struct cb_id id;
};

struct cn_callback_data {
	struct sk_buff *skb;
	void (*callback) (struct cn_msg *, struct netlink_skb_parms *);

	void *free;
};

struct cn_callback_entry {
	struct list_head callback_entry;
	struct work_struct work;
	struct cn_queue_dev *pdev;

	struct cn_callback_id id;
	struct cn_callback_data data;

	u32 seq, group;
};

struct cn_dev {
	struct cb_id id;

	u32 seq, groups;
	struct sock *nls;
	void (*input) (struct sk_buff *skb);

	struct cn_queue_dev *cbdev;
};

int cn_add_callback(struct cb_id *, char *, void (*callback) (struct cn_msg *, struct netlink_skb_parms *));
void cn_del_callback(struct cb_id *);
int cn_netlink_send(struct cn_msg *, u32, gfp_t);

int cn_queue_add_callback(struct cn_queue_dev *dev, char *name, struct cb_id *id, void (*callback)(struct cn_msg *, struct netlink_skb_parms *));
void cn_queue_del_callback(struct cn_queue_dev *dev, struct cb_id *id);

int queue_cn_work(struct cn_callback_entry *cbq, struct work_struct *work);

struct cn_queue_dev *cn_queue_alloc_dev(char *name, struct sock *);
void cn_queue_free_dev(struct cn_queue_dev *dev);

int cn_cb_equal(struct cb_id *, struct cb_id *);

void cn_queue_wrapper(struct work_struct *work);

#endif				/* __KERNEL__ */
#endif				/* __CONNECTOR_H */
