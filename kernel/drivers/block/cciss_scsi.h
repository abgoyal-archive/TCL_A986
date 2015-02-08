
#ifdef CONFIG_CISS_SCSI_TAPE
#ifndef _CCISS_SCSI_H_
#define _CCISS_SCSI_H_

#include <scsi/scsicam.h> /* possibly irrelevant, since we don't show disks */

		/* the scsi id of the adapter... */
#define SELF_SCSI_ID 15
		/* 15 is somewhat arbitrary, since the scsi-2 bus
		   that's presented by the driver to the OS is
		   fabricated.  The "real" scsi-3 bus the
		   hardware presents is fabricated too.
		   The actual, honest-to-goodness physical
		   bus that the devices are attached to is not
		   addressible natively, and may in fact turn
		   out to be not scsi at all. */

#define SCSI_CCISS_CAN_QUEUE 2


struct cciss_scsi_dev_t {
	int devtype;
	int bus, target, lun;		/* as presented to the OS */
	unsigned char scsi3addr[8];	/* as presented to the HW */
	unsigned char device_id[16];	/* from inquiry pg. 0x83 */
	unsigned char vendor[8];	/* bytes 8-15 of inquiry data */
	unsigned char model[16];	/* bytes 16-31 of inquiry data */
	unsigned char revision[4];	/* bytes 32-35 of inquiry data */
};

struct cciss_scsi_hba_t {
	char *name;
	int ndevices;
#define CCISS_MAX_SCSI_DEVS_PER_HBA 16
	struct cciss_scsi_dev_t dev[CCISS_MAX_SCSI_DEVS_PER_HBA];
};

#endif /* _CCISS_SCSI_H_ */
#endif /* CONFIG_CISS_SCSI_TAPE */
