

#ifndef _SYM53C416_H
#define _SYM53C416_H

#include <linux/types.h>

#define SYM53C416_SCSI_ID 7

static int sym53c416_detect(struct scsi_host_template *);
static const char *sym53c416_info(struct Scsi_Host *);
static int sym53c416_release(struct Scsi_Host *);
static int sym53c416_queuecommand(Scsi_Cmnd *, void (*done)(Scsi_Cmnd *));
static int sym53c416_host_reset(Scsi_Cmnd *);
static int sym53c416_bios_param(struct scsi_device *, struct block_device *,
		sector_t, int *);
static void sym53c416_setup(char *str, int *ints);
#endif
