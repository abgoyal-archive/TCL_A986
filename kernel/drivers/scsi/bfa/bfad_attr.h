

#ifndef __BFAD_ATTR_H__
#define __BFAD_ATTR_H__


struct Scsi_Host*
bfad_os_dev_to_shost(struct scsi_target *starget);

void
bfad_im_get_starget_port_id(struct scsi_target *starget);

void
bfad_im_get_starget_node_name(struct scsi_target *starget);

void
bfad_im_get_starget_port_name(struct scsi_target *starget);

void
bfad_im_get_host_port_id(struct Scsi_Host *shost);

struct Scsi_Host*
bfad_os_starget_to_shost(struct scsi_target *starget);


#endif /*  __BFAD_ATTR_H__ */
