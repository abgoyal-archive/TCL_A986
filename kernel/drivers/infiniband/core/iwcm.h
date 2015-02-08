
#ifndef IWCM_H
#define IWCM_H

enum iw_cm_state {
	IW_CM_STATE_IDLE,             /* unbound, inactive */
	IW_CM_STATE_LISTEN,           /* listen waiting for connect */
	IW_CM_STATE_CONN_RECV,        /* inbound waiting for user accept */
	IW_CM_STATE_CONN_SENT,        /* outbound waiting for peer accept */
	IW_CM_STATE_ESTABLISHED,      /* established */
	IW_CM_STATE_CLOSING,	      /* disconnect */
	IW_CM_STATE_DESTROYING        /* object being deleted */
};

struct iwcm_id_private {
	struct iw_cm_id	id;
	enum iw_cm_state state;
	unsigned long flags;
	struct ib_qp *qp;
	struct completion destroy_comp;
	wait_queue_head_t connect_wait;
	struct list_head work_list;
	spinlock_t lock;
	atomic_t refcount;
	struct list_head work_free_list;
};

#define IWCM_F_CALLBACK_DESTROY   1
#define IWCM_F_CONNECT_WAIT       2

#endif /* IWCM_H */
