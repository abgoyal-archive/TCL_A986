

#ifndef _SAS_ATA_H_
#define _SAS_ATA_H_

#include <linux/libata.h>
#include <scsi/libsas.h>

#ifdef CONFIG_SCSI_SAS_ATA

static inline int dev_is_sata(struct domain_device *dev)
{
	return (dev->rphy->identify.target_port_protocols & SAS_PROTOCOL_SATA);
}

int sas_ata_init_host_and_port(struct domain_device *found_dev,
			       struct scsi_target *starget);

void sas_ata_task_abort(struct sas_task *task);

#else


static inline int dev_is_sata(struct domain_device *dev)
{
	return 0;
}
static inline int sas_ata_init_host_and_port(struct domain_device *found_dev,
			       struct scsi_target *starget)
{
	return 0;
}
static inline void sas_ata_task_abort(struct sas_task *task)
{
}
#endif

#endif /* _SAS_ATA_H_ */
