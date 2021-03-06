


#ifndef RT2X00USB_H
#define RT2X00USB_H

#include <linux/usb.h>

#define to_usb_device_intf(d) \
({ \
	struct usb_interface *intf = to_usb_interface(d); \
	interface_to_usbdev(intf); \
})

#define USB_DEVICE_DATA(__ops)	.driver_info = (kernel_ulong_t)(__ops)

#define REGISTER_TIMEOUT		500
#define REGISTER_TIMEOUT_FIRMWARE	1000

#define REGISTER_TIMEOUT16(__datalen)	\
	( REGISTER_TIMEOUT * ((__datalen) / sizeof(u16)) )

#define REGISTER_TIMEOUT32(__datalen)	\
	( REGISTER_TIMEOUT * ((__datalen) / sizeof(u32)) )

#define CSR_CACHE_SIZE			64

#define USB_VENDOR_REQUEST	( USB_TYPE_VENDOR | USB_RECIP_DEVICE )
#define USB_VENDOR_REQUEST_IN	( USB_DIR_IN | USB_VENDOR_REQUEST )
#define USB_VENDOR_REQUEST_OUT	( USB_DIR_OUT | USB_VENDOR_REQUEST )

enum rt2x00usb_vendor_request {
	USB_DEVICE_MODE = 1,
	USB_SINGLE_WRITE = 2,
	USB_SINGLE_READ = 3,
	USB_MULTI_WRITE = 6,
	USB_MULTI_READ = 7,
	USB_EEPROM_WRITE = 8,
	USB_EEPROM_READ = 9,
	USB_LED_CONTROL = 10, /* RT73USB */
	USB_RX_CONTROL = 12,
};

enum rt2x00usb_mode_offset {
	USB_MODE_RESET = 1,
	USB_MODE_UNPLUG = 2,
	USB_MODE_FUNCTION = 3,
	USB_MODE_TEST = 4,
	USB_MODE_SLEEP = 7,	/* RT73USB */
	USB_MODE_FIRMWARE = 8,	/* RT73USB */
	USB_MODE_WAKEUP = 9,	/* RT73USB */
};

int rt2x00usb_vendor_request(struct rt2x00_dev *rt2x00dev,
			     const u8 request, const u8 requesttype,
			     const u16 offset, const u16 value,
			     void *buffer, const u16 buffer_length,
			     const int timeout);

int rt2x00usb_vendor_request_buff(struct rt2x00_dev *rt2x00dev,
				  const u8 request, const u8 requesttype,
				  const u16 offset, void *buffer,
				  const u16 buffer_length, const int timeout);

int rt2x00usb_vendor_req_buff_lock(struct rt2x00_dev *rt2x00dev,
				   const u8 request, const u8 requesttype,
				   const u16 offset, void *buffer,
				   const u16 buffer_length, const int timeout);

int rt2x00usb_vendor_request_large_buff(struct rt2x00_dev *rt2x00dev,
					const u8 request, const u8 requesttype,
					const u16 offset, const void *buffer,
					const u16 buffer_length,
					const int timeout);

static inline int rt2x00usb_vendor_request_sw(struct rt2x00_dev *rt2x00dev,
					      const u8 request,
					      const u16 offset,
					      const u16 value,
					      const int timeout)
{
	return rt2x00usb_vendor_request(rt2x00dev, request,
					USB_VENDOR_REQUEST_OUT, offset,
					value, NULL, 0, timeout);
}

static inline int rt2x00usb_eeprom_read(struct rt2x00_dev *rt2x00dev,
					__le16 *eeprom, const u16 length)
{
	return rt2x00usb_vendor_request(rt2x00dev, USB_EEPROM_READ,
					USB_VENDOR_REQUEST_IN, 0, 0,
					eeprom, length,
					REGISTER_TIMEOUT16(length));
}

static inline void rt2x00usb_register_read(struct rt2x00_dev *rt2x00dev,
					   const unsigned int offset,
					   u32 *value)
{
	__le32 reg;
	rt2x00usb_vendor_request_buff(rt2x00dev, USB_MULTI_READ,
				      USB_VENDOR_REQUEST_IN, offset,
				      &reg, sizeof(reg), REGISTER_TIMEOUT);
	*value = le32_to_cpu(reg);
}

static inline void rt2x00usb_register_read_lock(struct rt2x00_dev *rt2x00dev,
						const unsigned int offset,
						u32 *value)
{
	__le32 reg;
	rt2x00usb_vendor_req_buff_lock(rt2x00dev, USB_MULTI_READ,
				       USB_VENDOR_REQUEST_IN, offset,
				       &reg, sizeof(reg), REGISTER_TIMEOUT);
	*value = le32_to_cpu(reg);
}

static inline void rt2x00usb_register_multiread(struct rt2x00_dev *rt2x00dev,
						const unsigned int offset,
						void *value, const u32 length)
{
	rt2x00usb_vendor_request_buff(rt2x00dev, USB_MULTI_READ,
				      USB_VENDOR_REQUEST_IN, offset,
				      value, length,
				      REGISTER_TIMEOUT32(length));
}

static inline void rt2x00usb_register_write(struct rt2x00_dev *rt2x00dev,
					    const unsigned int offset,
					    u32 value)
{
	__le32 reg = cpu_to_le32(value);
	rt2x00usb_vendor_request_buff(rt2x00dev, USB_MULTI_WRITE,
				      USB_VENDOR_REQUEST_OUT, offset,
				      &reg, sizeof(reg), REGISTER_TIMEOUT);
}

static inline void rt2x00usb_register_write_lock(struct rt2x00_dev *rt2x00dev,
						 const unsigned int offset,
						 u32 value)
{
	__le32 reg = cpu_to_le32(value);
	rt2x00usb_vendor_req_buff_lock(rt2x00dev, USB_MULTI_WRITE,
				       USB_VENDOR_REQUEST_OUT, offset,
				       &reg, sizeof(reg), REGISTER_TIMEOUT);
}

static inline void rt2x00usb_register_multiwrite(struct rt2x00_dev *rt2x00dev,
						 const unsigned int offset,
						 const void *value,
						 const u32 length)
{
	rt2x00usb_vendor_request_buff(rt2x00dev, USB_MULTI_WRITE,
				      USB_VENDOR_REQUEST_OUT, offset,
				      (void *)value, length,
				      REGISTER_TIMEOUT32(length));
}

int rt2x00usb_regbusy_read(struct rt2x00_dev *rt2x00dev,
			   const unsigned int offset,
			   const struct rt2x00_field32 field,
			   u32 *reg);

void rt2x00usb_disable_radio(struct rt2x00_dev *rt2x00dev);

int rt2x00usb_write_tx_data(struct queue_entry *entry,
			    struct txentry_desc *txdesc);

struct queue_entry_priv_usb {
	struct urb *urb;
};

struct queue_entry_priv_usb_bcn {
	struct urb *urb;

	unsigned int guardian_data;
	struct urb *guardian_urb;
};

void rt2x00usb_kick_tx_queue(struct rt2x00_dev *rt2x00dev,
			     const enum data_queue_qid qid);

void rt2x00usb_kill_tx_queue(struct rt2x00_dev *rt2x00dev,
			      const enum data_queue_qid qid);

void rt2x00usb_clear_entry(struct queue_entry *entry);
int rt2x00usb_initialize(struct rt2x00_dev *rt2x00dev);
void rt2x00usb_uninitialize(struct rt2x00_dev *rt2x00dev);

int rt2x00usb_probe(struct usb_interface *usb_intf,
		    const struct usb_device_id *id);
void rt2x00usb_disconnect(struct usb_interface *usb_intf);
#ifdef CONFIG_PM
int rt2x00usb_suspend(struct usb_interface *usb_intf, pm_message_t state);
int rt2x00usb_resume(struct usb_interface *usb_intf);
#else
#define rt2x00usb_suspend	NULL
#define rt2x00usb_resume	NULL
#endif /* CONFIG_PM */

#endif /* RT2X00USB_H */
