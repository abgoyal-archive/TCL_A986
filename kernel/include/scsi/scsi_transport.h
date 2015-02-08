
#ifndef SCSI_TRANSPORT_H
#define SCSI_TRANSPORT_H

#include <linux/transport_class.h>
#include <linux/blkdev.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_device.h>

struct scsi_transport_template {
	/* the attribute containers */
	struct transport_container host_attrs;
	struct transport_container target_attrs;
	struct transport_container device_attrs;

	/*
	 * If set, called from sysfs and legacy procfs rescanning code.
	 */
	int (*user_scan)(struct Scsi_Host *, uint, uint, uint);

	/* The size of the specific transport attribute structure (a
	 * space of this size will be left at the end of the
	 * scsi_* structure */
	int	device_size;
	int	device_private_offset;
	int	target_size;
	int	target_private_offset;
	int	host_size;
	/* no private offset for the host; there's an alternative mechanism */

	/*
	 * True if the transport wants to use a host-based work-queue
	 */
	unsigned int create_work_queue : 1;

	/*
	 * Allows a transport to override the default error handler.
	 */
	void (* eh_strategy_handler)(struct Scsi_Host *);

	/*
	 * This is an optional routine that allows the transport to become
	 * involved when a scsi io timer fires. The return value tells the
	 * timer routine how to finish the io timeout handling:
	 * EH_HANDLED:		I fixed the error, please complete the command
	 * EH_RESET_TIMER:	I need more time, reset the timer and
	 *			begin counting again
	 * EH_NOT_HANDLED	Begin normal error recovery
	 */
	enum blk_eh_timer_return (*eh_timed_out)(struct scsi_cmnd *);

	/*
	 * Used as callback for the completion of i_t_nexus request
	 * for target drivers.
	 */
	int (* it_nexus_response)(struct Scsi_Host *, u64, int);

	/*
	 * Used as callback for the completion of task management
	 * request for target drivers.
	 */
	int (* tsk_mgmt_response)(struct Scsi_Host *, u64, u64, int);
};

#define transport_class_to_shost(tc) \
	dev_to_shost((tc)->parent)


static inline void
scsi_transport_reserve_target(struct scsi_transport_template * t, int space)
{
	BUG_ON(t->target_private_offset != 0);
	t->target_private_offset = ALIGN(t->target_size, sizeof(void *));
	t->target_size = t->target_private_offset + space;
}
static inline void
scsi_transport_reserve_device(struct scsi_transport_template * t, int space)
{
	BUG_ON(t->device_private_offset != 0);
	t->device_private_offset = ALIGN(t->device_size, sizeof(void *));
	t->device_size = t->device_private_offset + space;
}
static inline void *
scsi_transport_target_data(struct scsi_target *starget)
{
	struct Scsi_Host *shost = dev_to_shost(&starget->dev);
	return (u8 *)starget->starget_data
		+ shost->transportt->target_private_offset;

}
static inline void *
scsi_transport_device_data(struct scsi_device *sdev)
{
	struct Scsi_Host *shost = sdev->host;
	return (u8 *)sdev->sdev_data
		+ shost->transportt->device_private_offset;
}

#endif /* SCSI_TRANSPORT_H */
