

#ifndef __LINUX_IPMI_SMI_H
#define __LINUX_IPMI_SMI_H

#include <linux/ipmi_msgdefs.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>


/* Structure for the low-level drivers. */
typedef struct ipmi_smi *ipmi_smi_t;

struct ipmi_smi_msg {
	struct list_head link;

	long    msgid;
	void    *user_data;

	int           data_size;
	unsigned char data[IPMI_MAX_MSG_LENGTH];

	int           rsp_size;
	unsigned char rsp[IPMI_MAX_MSG_LENGTH];

	/* Will be called when the system is done with the message
	   (presumably to free it). */
	void (*done)(struct ipmi_smi_msg *msg);
};

struct ipmi_smi_handlers {
	struct module *owner;

	/* The low-level interface cannot start sending messages to
	   the upper layer until this function is called.  This may
	   not be NULL, the lower layer must take the interface from
	   this call. */
	int (*start_processing)(void       *send_info,
				ipmi_smi_t new_intf);

	/* Called to enqueue an SMI message to be sent.  This
	   operation is not allowed to fail.  If an error occurs, it
	   should report back the error in a received message.  It may
	   do this in the current call context, since no write locks
	   are held when this is run.  If the priority is > 0, the
	   message will go into a high-priority queue and be sent
	   first.  Otherwise, it goes into a normal-priority queue. */
	void (*sender)(void                *send_info,
		       struct ipmi_smi_msg *msg,
		       int                 priority);

	/* Called by the upper layer to request that we try to get
	   events from the BMC we are attached to. */
	void (*request_events)(void *send_info);

	/* Called when the interface should go into "run to
	   completion" mode.  If this call sets the value to true, the
	   interface should make sure that all messages are flushed
	   out and that none are pending, and any new requests are run
	   to completion immediately. */
	void (*set_run_to_completion)(void *send_info, int run_to_completion);

	/* Called to poll for work to do.  This is so upper layers can
	   poll for operations during things like crash dumps. */
	void (*poll)(void *send_info);

	/* Enable/disable firmware maintenance mode.  Note that this
	   is *not* the modes defined, this is simply an on/off
	   setting.  The message handler does the mode handling.  Note
	   that this is called from interrupt context, so it cannot
	   block. */
	void (*set_maintenance_mode)(void *send_info, int enable);

	/* Tell the handler that we are using it/not using it.  The
	   message handler get the modules that this handler belongs
	   to; this function lets the SMI claim any modules that it
	   uses.  These may be NULL if this is not required. */
	int (*inc_usecount)(void *send_info);
	void (*dec_usecount)(void *send_info);
};

struct ipmi_device_id {
	unsigned char device_id;
	unsigned char device_revision;
	unsigned char firmware_revision_1;
	unsigned char firmware_revision_2;
	unsigned char ipmi_version;
	unsigned char additional_device_support;
	unsigned int  manufacturer_id;
	unsigned int  product_id;
	unsigned char aux_firmware_revision[4];
	unsigned int  aux_firmware_revision_set : 1;
};

#define ipmi_version_major(v) ((v)->ipmi_version & 0xf)
#define ipmi_version_minor(v) ((v)->ipmi_version >> 4)

static inline int ipmi_demangle_device_id(const unsigned char *data,
					  unsigned int data_len,
					  struct ipmi_device_id *id)
{
	if (data_len < 9)
		return -EINVAL;
	if (data[0] != IPMI_NETFN_APP_RESPONSE << 2 ||
	    data[1] != IPMI_GET_DEVICE_ID_CMD)
		/* Strange, didn't get the response we expected. */
		return -EINVAL;
	if (data[2] != 0)
		/* That's odd, it shouldn't be able to fail. */
		return -EINVAL;

	data += 3;
	data_len -= 3;
	id->device_id = data[0];
	id->device_revision = data[1];
	id->firmware_revision_1 = data[2];
	id->firmware_revision_2 = data[3];
	id->ipmi_version = data[4];
	id->additional_device_support = data[5];
	if (data_len >= 11) {
		id->manufacturer_id = (data[6] | (data[7] << 8) |
				       (data[8] << 16));
		id->product_id = data[9] | (data[10] << 8);
	} else {
		id->manufacturer_id = 0;
		id->product_id = 0;
	}
	if (data_len >= 15) {
		memcpy(id->aux_firmware_revision, data+11, 4);
		id->aux_firmware_revision_set = 1;
	} else
		id->aux_firmware_revision_set = 0;

	return 0;
}

int ipmi_register_smi(struct ipmi_smi_handlers *handlers,
		      void                     *send_info,
		      struct ipmi_device_id    *device_id,
		      struct device            *dev,
		      const char               *sysfs_name,
		      unsigned char            slave_addr);

int ipmi_unregister_smi(ipmi_smi_t intf);

void ipmi_smi_msg_received(ipmi_smi_t          intf,
			   struct ipmi_smi_msg *msg);

/* The lower layer received a watchdog pre-timeout on interface. */
void ipmi_smi_watchdog_pretimeout(ipmi_smi_t intf);

struct ipmi_smi_msg *ipmi_alloc_smi_msg(void);
static inline void ipmi_free_smi_msg(struct ipmi_smi_msg *msg)
{
	msg->done(msg);
}

int ipmi_smi_add_proc_entry(ipmi_smi_t smi, char *name,
			    read_proc_t *read_proc,
			    void *data);

#endif /* __LINUX_IPMI_SMI_H */
