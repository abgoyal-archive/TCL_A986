
#ifndef IBMVSCSI_H
#define IBMVSCSI_H
#include <linux/types.h>
#include <linux/list.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include "viosrp.h"

struct scsi_cmnd;
struct Scsi_Host;

#define MAX_INDIRECT_BUFS 10

#define IBMVSCSI_MAX_REQUESTS_DEFAULT 100
#define IBMVSCSI_CMDS_PER_LUN_DEFAULT 16
#define IBMVSCSI_MAX_SECTORS_DEFAULT 256 /* 32 * 8 = default max I/O 32 pages */
#define IBMVSCSI_MAX_CMDS_PER_LUN 64

/* an RPA command/response transport queue */
struct crq_queue {
	struct viosrp_crq *msgs;
	int size, cur;
	dma_addr_t msg_token;
	spinlock_t lock;
};

/* a unit of work for the hosting partition */
struct srp_event_struct {
	union viosrp_iu *xfer_iu;
	struct scsi_cmnd *cmnd;
	struct list_head list;
	void (*done) (struct srp_event_struct *);
	struct viosrp_crq crq;
	struct ibmvscsi_host_data *hostdata;
	atomic_t free;
	union viosrp_iu iu;
	void (*cmnd_done) (struct scsi_cmnd *);
	struct completion comp;
	struct timer_list timer;
	union viosrp_iu *sync_srp;
	struct srp_direct_buf *ext_list;
	dma_addr_t ext_list_token;
};

/* a pool of event structs for use */
struct event_pool {
	struct srp_event_struct *events;
	u32 size;
	int next;
	union viosrp_iu *iu_storage;
	dma_addr_t iu_token;
};

/* all driver data associated with a host adapter */
struct ibmvscsi_host_data {
	atomic_t request_limit;
	int client_migrated;
	struct device *dev;
	struct event_pool pool;
	struct crq_queue queue;
	struct tasklet_struct srp_task;
	struct list_head sent;
	struct Scsi_Host *host;
	struct mad_adapter_info_data madapter_info;
	struct capabilities caps;
	dma_addr_t caps_addr;
	dma_addr_t adapter_info_addr;
};

/* routines for managing a command/response queue */
void ibmvscsi_handle_crq(struct viosrp_crq *crq,
			 struct ibmvscsi_host_data *hostdata);

struct ibmvscsi_ops {
	int (*init_crq_queue)(struct crq_queue *queue,
			      struct ibmvscsi_host_data *hostdata,
			      int max_requests);
	void (*release_crq_queue)(struct crq_queue *queue,
				  struct ibmvscsi_host_data *hostdata,
				  int max_requests);
	int (*reset_crq_queue)(struct crq_queue *queue,
			       struct ibmvscsi_host_data *hostdata);
	int (*reenable_crq_queue)(struct crq_queue *queue,
				  struct ibmvscsi_host_data *hostdata);
	int (*send_crq)(struct ibmvscsi_host_data *hostdata,
		       u64 word1, u64 word2);
	int (*resume) (struct ibmvscsi_host_data *hostdata);
};

extern struct ibmvscsi_ops iseriesvscsi_ops;
extern struct ibmvscsi_ops rpavscsi_ops;

#endif				/* IBMVSCSI_H */
