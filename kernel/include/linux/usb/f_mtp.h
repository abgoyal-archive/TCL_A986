

#ifndef __LINUX_USB_F_MTP_H
#define __LINUX_USB_F_MTP_H

/* Constants for MTP_SET_INTERFACE_MODE */
#define MTP_INTERFACE_MODE_MTP  0
#define MTP_INTERFACE_MODE_PTP  1


struct mtp_file_range {
	/* file descriptor for file to transfer */
	int			fd;
	/* offset in file for start of transfer */
	loff_t  	offset;
	/* number of bytes to transfer */
	size_t		length;
};

struct mtp_event {
	/* size of the event */
	size_t		length;
	/* event data to send */
	void  		*data;
};

/* Sends the specified file range to the host */
#define MTP_SEND_FILE              _IOW('M', 0, struct mtp_file_range)
#define MTP_RECEIVE_FILE           _IOW('M', 1, struct mtp_file_range)
/* Sets the driver mode to either MTP or PTP */
#define MTP_SET_INTERFACE_MODE     _IOW('M', 2, int)
/* Sends an event to the host via the interrupt endpoint */
#define MTP_SEND_EVENT             _IOW('M', 3, struct mtp_event)

#endif /* __LINUX_USB_F_MTP_H */
