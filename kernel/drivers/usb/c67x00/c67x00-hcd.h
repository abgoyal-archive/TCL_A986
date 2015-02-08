

#ifndef _USB_C67X00_HCD_H
#define _USB_C67X00_HCD_H

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include "c67x00.h"



#define TOTAL_FRAME_BW		12000
#define DEFAULT_EOT		2250

#define MAX_FRAME_BW_STD	(TOTAL_FRAME_BW - DEFAULT_EOT)
#define MAX_FRAME_BW_ISO	2400

#define MAX_PERIODIC_BW(full_bw)	full_bw

/* -------------------------------------------------------------------------- */

struct c67x00_hcd {
	spinlock_t lock;
	struct c67x00_sie *sie;
	unsigned int low_speed_ports;	/* bitmask of low speed ports */
	unsigned int urb_count;
	unsigned int urb_iso_count;

	struct list_head list[4];	/* iso, int, ctrl, bulk */
#if PIPE_BULK != 3
#error "Sanity check failed, this code presumes PIPE_... to range from 0 to 3"
#endif

	/* USB bandwidth allocated to td_list */
	int bandwidth_allocated;
	/* USB bandwidth allocated for isoc/int transfer */
	int periodic_bw_allocated;
	struct list_head td_list;
	int max_frame_bw;

	u16 td_base_addr;
	u16 buf_base_addr;
	u16 next_td_addr;
	u16 next_buf_addr;

	struct tasklet_struct tasklet;

	struct completion endpoint_disable;

	u16 current_frame;
	u16 last_frame;
};

static inline struct c67x00_hcd *hcd_to_c67x00_hcd(struct usb_hcd *hcd)
{
	return (struct c67x00_hcd *)(hcd->hcd_priv);
}

static inline struct usb_hcd *c67x00_hcd_to_hcd(struct c67x00_hcd *c67x00)
{
	return container_of((void *)c67x00, struct usb_hcd, hcd_priv);
}


int c67x00_hcd_probe(struct c67x00_sie *sie);
void c67x00_hcd_remove(struct c67x00_sie *sie);

int c67x00_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags);
int c67x00_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status);
void c67x00_endpoint_disable(struct usb_hcd *hcd,
			     struct usb_host_endpoint *ep);

void c67x00_hcd_msg_received(struct c67x00_sie *sie, u16 msg);
void c67x00_sched_kick(struct c67x00_hcd *c67x00);
int c67x00_sched_start_scheduler(struct c67x00_hcd *c67x00);
void c67x00_sched_stop_scheduler(struct c67x00_hcd *c67x00);

#define c67x00_hcd_dev(x)	(c67x00_hcd_to_hcd(x)->self.controller)

#endif				/* _USB_C67X00_HCD_H */
