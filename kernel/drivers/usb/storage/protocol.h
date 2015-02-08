

#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

/* Protocol handling routines */
extern void usb_stor_pad12_command(struct scsi_cmnd*, struct us_data*);
extern void usb_stor_ufi_command(struct scsi_cmnd*, struct us_data*);
extern void usb_stor_transparent_scsi_command(struct scsi_cmnd*,
		struct us_data*);

/* struct scsi_cmnd transfer buffer access utilities */
enum xfer_buf_dir	{TO_XFER_BUF, FROM_XFER_BUF};

extern unsigned int usb_stor_access_xfer_buf(unsigned char *buffer,
	unsigned int buflen, struct scsi_cmnd *srb, struct scatterlist **,
	unsigned int *offset, enum xfer_buf_dir dir);

extern void usb_stor_set_xfer_buf(unsigned char *buffer,
	unsigned int buflen, struct scsi_cmnd *srb);
#endif
