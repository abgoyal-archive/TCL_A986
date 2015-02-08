


#ifndef __BFA_HCB_IOIM_MACROS_H__
#define __BFA_HCB_IOIM_MACROS_H__

#include <bfa_os_inc.h>
#include "bfad_im_compat.h"

#define SIMPLE_Q    0
#define HEAD_OF_Q   1
#define ORDERED_Q   2
#define ACA_Q       4
#define UNTAGGED    5

static inline lun_t
bfad_int_to_lun(u32 luno)
{
	union {
		u16        scsi_lun[4];
		lun_t           bfa_lun;
	} lun;

	lun.bfa_lun     = 0;
	lun.scsi_lun[0] = bfa_os_htons(luno);

	return lun.bfa_lun;
}

#define bfa_cb_ioim_get_lun(__dio)	\
	bfad_int_to_lun(((struct scsi_cmnd *)__dio)->device->lun)

static inline u8 *
bfa_cb_ioim_get_cdb(struct bfad_ioim_s *dio)
{
	struct scsi_cmnd *cmnd = (struct scsi_cmnd *)dio;

	return (u8 *) cmnd->cmnd;
}

static inline enum fcp_iodir
bfa_cb_ioim_get_iodir(struct bfad_ioim_s *dio)
{
	struct scsi_cmnd *cmnd = (struct scsi_cmnd *)dio;
	enum dma_data_direction dmadir;

	dmadir = cmnd->sc_data_direction;
	if (dmadir == DMA_TO_DEVICE)
		return FCP_IODIR_WRITE;
	else if (dmadir == DMA_FROM_DEVICE)
		return FCP_IODIR_READ;
	else
		return FCP_IODIR_NONE;
}

static inline u32
bfa_cb_ioim_get_size(struct bfad_ioim_s *dio)
{
	struct scsi_cmnd *cmnd = (struct scsi_cmnd *)dio;

	return scsi_bufflen(cmnd);
}

static inline u8
bfa_cb_ioim_get_timeout(struct bfad_ioim_s *dio)
{
	struct scsi_cmnd *cmnd = (struct scsi_cmnd *)dio;
	/*
	 * TBD: need a timeout for scsi passthru
	 */
	if (cmnd->device->host == NULL)
		return 4;

	return 0;
}

static inline u8
bfa_cb_ioim_get_crn(struct bfad_ioim_s *dio)
{
	return 0;
}

static inline u8
bfa_cb_ioim_get_priority(struct bfad_ioim_s *dio)
{
	return 0;
}

static inline u8
bfa_cb_ioim_get_taskattr(struct bfad_ioim_s *dio)
{
	struct scsi_cmnd *cmnd = (struct scsi_cmnd *)dio;
	u8         task_attr = UNTAGGED;

	if (cmnd->device->tagged_supported) {
		switch (cmnd->tag) {
		case HEAD_OF_QUEUE_TAG:
			task_attr = HEAD_OF_Q;
			break;
		case ORDERED_QUEUE_TAG:
			task_attr = ORDERED_Q;
			break;
		default:
			task_attr = SIMPLE_Q;
			break;
		}
	}

	return task_attr;
}

static inline u8
bfa_cb_ioim_get_cdblen(struct bfad_ioim_s *dio)
{
	struct scsi_cmnd *cmnd = (struct scsi_cmnd *)dio;

	return cmnd->cmd_len;
}



#endif /* __BFA_HCB_IOIM_MACROS_H__ */
