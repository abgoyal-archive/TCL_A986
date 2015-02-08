

#ifndef __IEEE1394_IOCTL_H
#define __IEEE1394_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* DV1394 Gets 10 */

#define DV1394_IOC_INIT			_IOW('#', 0x06, struct dv1394_init)

/* Stop transmitting video and free the ringbuffer */
#define DV1394_IOC_SHUTDOWN		_IO ('#', 0x07)

#define DV1394_IOC_SUBMIT_FRAMES	_IO ('#', 0x08)

#define DV1394_IOC_WAIT_FRAMES		_IO ('#', 0x09)

#define DV1394_IOC_RECEIVE_FRAMES	_IO ('#', 0x0a)

/* Tell card to start receiving DMA */
#define DV1394_IOC_START_RECEIVE	_IO ('#', 0x0b)

/* Pass a struct dv1394_status* as the parameter */
#define DV1394_IOC_GET_STATUS		_IOR('#', 0x0c, struct dv1394_status)


/* Video1394 Gets 10 */

#define VIDEO1394_IOC_LISTEN_CHANNEL		\
	_IOWR('#', 0x10, struct video1394_mmap)
#define VIDEO1394_IOC_UNLISTEN_CHANNEL		\
	_IOW ('#', 0x11, int)
#define VIDEO1394_IOC_LISTEN_QUEUE_BUFFER	\
	_IOW ('#', 0x12, struct video1394_wait)
#define VIDEO1394_IOC_LISTEN_WAIT_BUFFER	\
	_IOWR('#', 0x13, struct video1394_wait)
#define VIDEO1394_IOC_TALK_CHANNEL		\
	_IOWR('#', 0x14, struct video1394_mmap)
#define VIDEO1394_IOC_UNTALK_CHANNEL		\
	_IOW ('#', 0x15, int)
#define VIDEO1394_IOC_TALK_QUEUE_BUFFER 	\
	_IOW ('#', 0x16, size_t)
#define VIDEO1394_IOC_TALK_WAIT_BUFFER		\
	_IOW ('#', 0x17, struct video1394_wait)
#define VIDEO1394_IOC_LISTEN_POLL_BUFFER	\
	_IOWR('#', 0x18, struct video1394_wait)


/* Raw1394's ISO interface */
#define RAW1394_IOC_ISO_XMIT_INIT		\
	_IOW ('#', 0x1a, struct raw1394_iso_status)
#define RAW1394_IOC_ISO_RECV_INIT		\
	_IOWR('#', 0x1b, struct raw1394_iso_status)
#define RAW1394_IOC_ISO_RECV_START		\
	_IOC (_IOC_WRITE, '#', 0x1c, sizeof(int) * 3)
#define RAW1394_IOC_ISO_XMIT_START		\
	_IOC (_IOC_WRITE, '#', 0x1d, sizeof(int) * 2)
#define RAW1394_IOC_ISO_XMIT_RECV_STOP		\
	_IO  ('#', 0x1e)
#define RAW1394_IOC_ISO_GET_STATUS		\
	_IOR ('#', 0x1f, struct raw1394_iso_status)
#define RAW1394_IOC_ISO_SHUTDOWN		\
	_IO  ('#', 0x20)
#define RAW1394_IOC_ISO_QUEUE_ACTIVITY		\
	_IO  ('#', 0x21)
#define RAW1394_IOC_ISO_RECV_LISTEN_CHANNEL	\
	_IOW ('#', 0x22, unsigned char)
#define RAW1394_IOC_ISO_RECV_UNLISTEN_CHANNEL	\
	_IOW ('#', 0x23, unsigned char)
#define RAW1394_IOC_ISO_RECV_SET_CHANNEL_MASK	\
	_IOW ('#', 0x24, __u64)
#define RAW1394_IOC_ISO_RECV_PACKETS		\
	_IOW ('#', 0x25, struct raw1394_iso_packets)
#define RAW1394_IOC_ISO_RECV_RELEASE_PACKETS	\
	_IOW ('#', 0x26, unsigned int)
#define RAW1394_IOC_ISO_XMIT_PACKETS		\
	_IOW ('#', 0x27, struct raw1394_iso_packets)
#define RAW1394_IOC_ISO_XMIT_SYNC		\
	_IO  ('#', 0x28)
#define RAW1394_IOC_ISO_RECV_FLUSH		\
	_IO  ('#', 0x29)
#define RAW1394_IOC_GET_CYCLE_TIMER		\
	_IOR ('#', 0x30, struct raw1394_cycle_timer)

#endif /* __IEEE1394_IOCTL_H */
