

#ifndef __LH7A40X_H_
#define __LH7A40X_H_

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <asm/byteorder.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <mach/hardware.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>


#define USB_FA					0x80000200	// function address register
#define USB_PM					0x80000204	// power management register

#define USB_IN_INT				0x80000208	// IN interrupt register bank (EP0-EP3)
#define USB_OUT_INT				0x80000210	// OUT interrupt register bank (EP2)
#define USB_INT					0x80000218	// interrupt register bank

#define USB_IN_INT_EN			0x8000021C	// IN interrupt enable register bank
#define USB_OUT_INT_EN			0x80000224	// OUT interrupt enable register bank
#define USB_INT_EN				0x8000022C	// USB interrupt enable register bank

#define USB_FRM_NUM1			0x80000230	// Frame number1 register
#define USB_FRM_NUM2			0x80000234	// Frame number2 register
#define USB_INDEX				0x80000238	// index register

#define USB_IN_MAXP				0x80000240	// IN MAXP register
#define USB_IN_CSR1				0x80000244	// IN CSR1 register/EP0 CSR register
#define USB_EP0_CSR				0x80000244	// IN CSR1 register/EP0 CSR register
#define USB_IN_CSR2				0x80000248	// IN CSR2 register
#define USB_OUT_MAXP			0x8000024C	// OUT MAXP register

#define USB_OUT_CSR1			0x80000250	// OUT CSR1 register
#define USB_OUT_CSR2			0x80000254	// OUT CSR2 register
#define USB_OUT_FIFO_WC1		0x80000258	// OUT FIFO write count1 register
#define USB_OUT_FIFO_WC2		0x8000025C	// OUT FIFO write count2 register

#define USB_RESET				0x8000044C	// USB reset register

#define	USB_EP0_FIFO			0x80000280
#define	USB_EP1_FIFO			0x80000284
#define	USB_EP2_FIFO			0x80000288
#define	USB_EP3_FIFO			0x8000028c

#define USB_RESET_APB			(1<<1)	//resets USB APB control side WRITE
#define USB_RESET_IO			(1<<0)	//resets USB IO side WRITE

#define USB_FA_ADDR_UPDATE		(1<<7)
#define USB_FA_FUNCTION_ADDR	(0x7F)

#define PM_USB_DCP				(1<<5)
#define PM_USB_ENABLE			(1<<4)
#define PM_USB_RESET			(1<<3)
#define PM_UC_RESUME			(1<<2)
#define PM_SUSPEND_MODE			(1<<1)
#define PM_ENABLE_SUSPEND		(1<<0)

#define USB_IN_INT_EP3				(1<<3)
#define USB_IN_INT_EP1				(1<<1)
#define USB_IN_INT_EP0				(1<<0)

#define USB_OUT_INT_EP2				(1<<2)

#define USB_INT_RESET_INT			(1<<2)
#define USB_INT_RESUME_INT			(1<<1)
#define USB_INT_SUSPEND_INT			(1<<0)

#define USB_INT_EN_USB_RESET_INTER		(1<<2)
#define USB_INT_EN_RESUME_INTER			(1<<1)
#define USB_INT_EN_SUSPEND_INTER		(1<<0)

#define USB_IN_CSR1_CLR_DATA_TOGGLE		(1<<6)
#define USB_IN_CSR1_SENT_STALL			(1<<5)
#define USB_IN_CSR1_SEND_STALL			(1<<4)
#define USB_IN_CSR1_FIFO_FLUSH			(1<<3)
#define USB_IN_CSR1_FIFO_NOT_EMPTY		(1<<1)
#define USB_IN_CSR1_IN_PKT_RDY			(1<<0)

#define USB_IN_CSR2_AUTO_SET			(1<<7)
#define USB_IN_CSR2_USB_DMA_EN			(1<<4)

#define USB_OUT_CSR1_CLR_DATA_REG		(1<<7)
#define USB_OUT_CSR1_SENT_STALL			(1<<6)
#define USB_OUT_CSR1_SEND_STALL			(1<<5)
#define USB_OUT_CSR1_FIFO_FLUSH			(1<<4)
#define USB_OUT_CSR1_FIFO_FULL			(1<<1)
#define USB_OUT_CSR1_OUT_PKT_RDY		(1<<0)

#define USB_OUT_CSR2_AUTO_CLR			(1<<7)
#define USB_OUT_CSR2_USB_DMA_EN			(1<<4)

#define EP0_CLR_SETUP_END		(1<<7)	/* Clear "Setup Ends" Bit (w) */
#define EP0_CLR_OUT				(1<<6)	/* Clear "Out packet ready" Bit (w) */
#define EP0_SEND_STALL			(1<<5)	/* Send STALL Handshake (rw) */
#define EP0_SETUP_END			(1<<4)	/* Setup Ends (r) */

#define EP0_DATA_END			(1<<3)	/* Data end (rw) */
#define EP0_SENT_STALL			(1<<2)	/* Sent Stall Handshake (r) */
#define EP0_IN_PKT_RDY			(1<<1)	/* In packet ready (rw) */
#define EP0_OUT_PKT_RDY			(1<<0)	/* Out packet ready (r) */

/* general CSR */
#define OUT_PKT_RDY		(1<<0)
#define IN_PKT_RDY		(1<<0)

#define USB_OUT_MAXP_MAXP			(0xF)
#define USB_IN_MAXP_MAXP			(0xF)

// Max packet size
//#define EP0_PACKETSIZE        0x10
#define EP0_PACKETSIZE  	0x8
#define EP0_MAXPACKETSIZE  	0x10

#define UDC_MAX_ENDPOINTS       4

#define WAIT_FOR_SETUP          0
#define DATA_STATE_XMIT         1
#define DATA_STATE_NEED_ZLP     2
#define WAIT_FOR_OUT_STATUS     3
#define DATA_STATE_RECV         4

/* ********************************************************************************************* */

typedef enum ep_type {
	ep_control, ep_bulk_in, ep_bulk_out, ep_interrupt
} ep_type_t;

struct lh7a40x_ep {
	struct usb_ep ep;
	struct lh7a40x_udc *dev;

	const struct usb_endpoint_descriptor *desc;
	struct list_head queue;
	unsigned long pio_irqs;

	u8 stopped;
	u8 bEndpointAddress;
	u8 bmAttributes;

	ep_type_t ep_type;
	u32 fifo;
	u32 csr1;
	u32 csr2;
};

struct lh7a40x_request {
	struct usb_request req;
	struct list_head queue;
};

struct lh7a40x_udc {
	struct usb_gadget gadget;
	struct usb_gadget_driver *driver;
	struct device *dev;
	spinlock_t lock;

	int ep0state;
	struct lh7a40x_ep ep[UDC_MAX_ENDPOINTS];

	unsigned char usb_address;

	unsigned req_pending:1, req_std:1, req_config:1;
};

extern struct lh7a40x_udc *the_controller;

#define ep_is_in(EP) 		(((EP)->bEndpointAddress&USB_DIR_IN)==USB_DIR_IN)
#define ep_index(EP) 		((EP)->bEndpointAddress&0xF)
#define ep_maxpacket(EP) 	((EP)->ep.maxpacket)

#endif
