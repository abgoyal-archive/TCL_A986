
#ifndef SCSI_NETLINK_H
#define SCSI_NETLINK_H

#include <linux/netlink.h>



/* Single Netlink Message type to send all SCSI Transport messages */
#define SCSI_TRANSPORT_MSG		NLMSG_MIN_TYPE + 1

/* SCSI Transport Broadcast Groups */
	/* leaving groups 0 and 1 unassigned */
#define SCSI_NL_GRP_FC_EVENTS		(1<<2)		/* Group 2 */
#define SCSI_NL_GRP_CNT			3


/* SCSI_TRANSPORT_MSG event message header */
struct scsi_nl_hdr {
	uint8_t version;
	uint8_t transport;
	uint16_t magic;
	uint16_t msgtype;
	uint16_t msglen;
} __attribute__((aligned(sizeof(uint64_t))));

/* scsi_nl_hdr->version value */
#define SCSI_NL_VERSION				1

/* scsi_nl_hdr->magic value */
#define SCSI_NL_MAGIC				0xA1B2

/* scsi_nl_hdr->transport value */
#define SCSI_NL_TRANSPORT			0
#define SCSI_NL_TRANSPORT_FC			1
#define SCSI_NL_MAX_TRANSPORTS			2

/* Transport-based scsi_nl_hdr->msgtype values are defined in each transport */

	/* kernel -> user */
#define SCSI_NL_SHOST_VENDOR			0x0001
	/* user -> kernel */
/* SCSI_NL_SHOST_VENDOR msgtype is kernel->user and user->kernel */



/* macro to round up message lengths to 8byte boundary */
#define SCSI_NL_MSGALIGN(len)		(((len) + 7) & ~7)


struct scsi_nl_host_vendor_msg {
	struct scsi_nl_hdr snlh;		/* must be 1st element ! */
	uint64_t vendor_id;
	uint16_t host_no;
	uint16_t vmsg_datalen;
} __attribute__((aligned(sizeof(uint64_t))));


#define SCSI_NL_VID_TYPE_SHIFT		56
#define SCSI_NL_VID_TYPE_MASK		((u64)0xFF << SCSI_NL_VID_TYPE_SHIFT)
#define SCSI_NL_VID_TYPE_PCI		((u64)0x01 << SCSI_NL_VID_TYPE_SHIFT)
#define SCSI_NL_VID_ID_MASK		(~ SCSI_NL_VID_TYPE_MASK)


#define INIT_SCSI_NL_HDR(hdr, t, mtype, mlen)			\
	{							\
	(hdr)->version = SCSI_NL_VERSION;			\
	(hdr)->transport = t;					\
	(hdr)->magic = SCSI_NL_MAGIC;				\
	(hdr)->msgtype = mtype;					\
	(hdr)->msglen = mlen;					\
	}


#ifdef __KERNEL__

#include <scsi/scsi_host.h>

/* Exported Kernel Interfaces */
int scsi_nl_add_transport(u8 tport,
	 int (*msg_handler)(struct sk_buff *),
	void (*event_handler)(struct notifier_block *, unsigned long, void *));
void scsi_nl_remove_transport(u8 tport);

int scsi_nl_add_driver(u64 vendor_id, struct scsi_host_template *hostt,
	int (*nlmsg_handler)(struct Scsi_Host *shost, void *payload,
				 u32 len, u32 pid),
	void (*nlevt_handler)(struct notifier_block *nb,
				 unsigned long event, void *notify_ptr));
void scsi_nl_remove_driver(u64 vendor_id);

void scsi_nl_send_transport_msg(u32 pid, struct scsi_nl_hdr *hdr);
int scsi_nl_send_vendor_msg(u32 pid, unsigned short host_no, u64 vendor_id,
			 char *data_buf, u32 data_len);

#endif /* __KERNEL__ */

#endif /* SCSI_NETLINK_H */

