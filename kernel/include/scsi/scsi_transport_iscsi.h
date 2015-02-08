
#ifndef SCSI_TRANSPORT_ISCSI_H
#define SCSI_TRANSPORT_ISCSI_H

#include <linux/device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <scsi/iscsi_if.h>

struct scsi_transport_template;
struct iscsi_transport;
struct iscsi_endpoint;
struct Scsi_Host;
struct iscsi_cls_conn;
struct iscsi_conn;
struct iscsi_task;
struct sockaddr;

struct iscsi_transport {
	struct module *owner;
	char *name;
	unsigned int caps;
	/* LLD sets this to indicate what values it can export to sysfs */
	uint64_t param_mask;
	uint64_t host_param_mask;
	struct iscsi_cls_session *(*create_session) (struct iscsi_endpoint *ep,
					uint16_t cmds_max, uint16_t qdepth,
					uint32_t sn);
	void (*destroy_session) (struct iscsi_cls_session *session);
	struct iscsi_cls_conn *(*create_conn) (struct iscsi_cls_session *sess,
				uint32_t cid);
	int (*bind_conn) (struct iscsi_cls_session *session,
			  struct iscsi_cls_conn *cls_conn,
			  uint64_t transport_eph, int is_leading);
	int (*start_conn) (struct iscsi_cls_conn *conn);
	void (*stop_conn) (struct iscsi_cls_conn *conn, int flag);
	void (*destroy_conn) (struct iscsi_cls_conn *conn);
	int (*set_param) (struct iscsi_cls_conn *conn, enum iscsi_param param,
			  char *buf, int buflen);
	int (*get_conn_param) (struct iscsi_cls_conn *conn,
			       enum iscsi_param param, char *buf);
	int (*get_session_param) (struct iscsi_cls_session *session,
				  enum iscsi_param param, char *buf);
	int (*get_host_param) (struct Scsi_Host *shost,
				enum iscsi_host_param param, char *buf);
	int (*set_host_param) (struct Scsi_Host *shost,
			       enum iscsi_host_param param, char *buf,
			       int buflen);
	int (*send_pdu) (struct iscsi_cls_conn *conn, struct iscsi_hdr *hdr,
			 char *data, uint32_t data_size);
	void (*get_stats) (struct iscsi_cls_conn *conn,
			   struct iscsi_stats *stats);

	int (*init_task) (struct iscsi_task *task);
	int (*xmit_task) (struct iscsi_task *task);
	void (*cleanup_task) (struct iscsi_task *task);

	int (*alloc_pdu) (struct iscsi_task *task, uint8_t opcode);
	int (*xmit_pdu) (struct iscsi_task *task);
	int (*init_pdu) (struct iscsi_task *task, unsigned int offset,
			 unsigned int count);
	void (*parse_pdu_itt) (struct iscsi_conn *conn, itt_t itt,
			       int *index, int *age);

	void (*session_recovery_timedout) (struct iscsi_cls_session *session);
	struct iscsi_endpoint *(*ep_connect) (struct Scsi_Host *shost,
					      struct sockaddr *dst_addr,
					      int non_blocking);
	int (*ep_poll) (struct iscsi_endpoint *ep, int timeout_ms);
	void (*ep_disconnect) (struct iscsi_endpoint *ep);
	int (*tgt_dscvr) (struct Scsi_Host *shost, enum iscsi_tgt_dscvr type,
			  uint32_t enable, struct sockaddr *dst_addr);
	int (*set_path) (struct Scsi_Host *shost, struct iscsi_path *params);
};

extern struct scsi_transport_template *iscsi_register_transport(struct iscsi_transport *tt);
extern int iscsi_unregister_transport(struct iscsi_transport *tt);

extern void iscsi_conn_error_event(struct iscsi_cls_conn *conn,
				   enum iscsi_err error);
extern int iscsi_recv_pdu(struct iscsi_cls_conn *conn, struct iscsi_hdr *hdr,
			  char *data, uint32_t data_size);

extern int iscsi_offload_mesg(struct Scsi_Host *shost,
			      struct iscsi_transport *transport, uint32_t type,
			      char *data, uint16_t data_size);

struct iscsi_cls_conn {
	struct list_head conn_list;	/* item in connlist */
	void *dd_data;			/* LLD private data */
	struct iscsi_transport *transport;
	uint32_t cid;			/* connection id */

	int active;			/* must be accessed with the connlock */
	struct device dev;		/* sysfs transport/container device */
};

#define iscsi_dev_to_conn(_dev) \
	container_of(_dev, struct iscsi_cls_conn, dev)

#define iscsi_conn_to_session(_conn) \
	iscsi_dev_to_session(_conn->dev.parent)

/* iscsi class session state */
enum {
	ISCSI_SESSION_LOGGED_IN,
	ISCSI_SESSION_FAILED,
	ISCSI_SESSION_FREE,
};

#define ISCSI_MAX_TARGET -1

struct iscsi_cls_session {
	struct list_head sess_list;		/* item in session_list */
	struct iscsi_transport *transport;
	spinlock_t lock;
	struct work_struct block_work;
	struct work_struct unblock_work;
	struct work_struct scan_work;
	struct work_struct unbind_work;

	/* recovery fields */
	int recovery_tmo;
	struct delayed_work recovery_work;

	unsigned int target_id;

	int state;
	int sid;				/* session id */
	void *dd_data;				/* LLD private data */
	struct device dev;	/* sysfs transport/container device */
};

#define iscsi_dev_to_session(_dev) \
	container_of(_dev, struct iscsi_cls_session, dev)

#define iscsi_session_to_shost(_session) \
	dev_to_shost(_session->dev.parent)

#define starget_to_session(_stgt) \
	iscsi_dev_to_session(_stgt->dev.parent)

struct iscsi_cls_host {
	atomic_t nr_scans;
	struct mutex mutex;
};

extern void iscsi_host_for_each_session(struct Scsi_Host *shost,
				void (*fn)(struct iscsi_cls_session *));

struct iscsi_endpoint {
	void *dd_data;			/* LLD private data */
	struct device dev;
	uint64_t id;
};

#define iscsi_cls_session_printk(prefix, _cls_session, fmt, a...) \
	dev_printk(prefix, &(_cls_session)->dev, fmt, ##a)

#define iscsi_cls_conn_printk(prefix, _cls_conn, fmt, a...) \
	dev_printk(prefix, &(_cls_conn)->dev, fmt, ##a)

extern int iscsi_session_chkready(struct iscsi_cls_session *session);
extern struct iscsi_cls_session *iscsi_alloc_session(struct Scsi_Host *shost,
				struct iscsi_transport *transport, int dd_size);
extern int iscsi_add_session(struct iscsi_cls_session *session,
			     unsigned int target_id);
extern int iscsi_session_event(struct iscsi_cls_session *session,
			       enum iscsi_uevent_e event);
extern struct iscsi_cls_session *iscsi_create_session(struct Scsi_Host *shost,
						struct iscsi_transport *t,
						int dd_size,
						unsigned int target_id);
extern void iscsi_remove_session(struct iscsi_cls_session *session);
extern void iscsi_free_session(struct iscsi_cls_session *session);
extern int iscsi_destroy_session(struct iscsi_cls_session *session);
extern struct iscsi_cls_conn *iscsi_create_conn(struct iscsi_cls_session *sess,
						int dd_size, uint32_t cid);
extern int iscsi_destroy_conn(struct iscsi_cls_conn *conn);
extern void iscsi_unblock_session(struct iscsi_cls_session *session);
extern void iscsi_block_session(struct iscsi_cls_session *session);
extern int iscsi_scan_finished(struct Scsi_Host *shost, unsigned long time);
extern struct iscsi_endpoint *iscsi_create_endpoint(int dd_size);
extern void iscsi_destroy_endpoint(struct iscsi_endpoint *ep);
extern struct iscsi_endpoint *iscsi_lookup_endpoint(u64 handle);

#endif
