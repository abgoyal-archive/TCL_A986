
#ifndef IDA_IOCTL_H
#define IDA_IOCTL_H

#include "ida_cmd.h"
#include "cpqarray.h"

#define IDAGETDRVINFO		0x27272828
#define IDAPASSTHRU		0x28282929
#define IDAGETCTLRSIG		0x29293030
#define IDAREVALIDATEVOLS	0x30303131
#define IDADRIVERVERSION	0x31313232
#define IDAGETPCIINFO		0x32323333

typedef struct _ida_pci_info_struct
{
	unsigned char 	bus;
	unsigned char 	dev_fn;
	__u32 		board_id;
} ida_pci_info_struct;

#define UNITVALID	0x80
typedef struct {
	__u8	cmd;
	__u8	rcode;
	__u8	unit;
	__u32	blk;
	__u16	blk_cnt;

/* currently, sg_cnt is assumed to be 1: only the 0th element of sg is used */
	struct {
		void	__user *addr;
		size_t	size;
	} sg[SG_MAX];
	int	sg_cnt;

	union ctlr_cmds {
		drv_info_t		drv;
		unsigned char		buf[1024];

		id_ctlr_t		id_ctlr;
		drv_param_t		drv_param;
		id_log_drv_t		id_log_drv;
		id_log_drv_ext_t	id_log_drv_ext;
		sense_log_drv_stat_t	sense_log_drv_stat;
		id_phys_drv_t		id_phys_drv;
		blink_drv_leds_t	blink_drv_leds;
		sense_blink_leds_t	sense_blink_leds;
		config_t		config;
		reorder_log_drv_t	reorder_log_drv;
		label_log_drv_t		label_log_drv;
		surf_delay_t		surf_delay;
		overhead_delay_t	overhead_delay;
		mp_delay_t		mp_delay;
		scsi_param_t		scsi_param;
	} c;
} ida_ioctl_t;

#endif /* IDA_IOCTL_H */
