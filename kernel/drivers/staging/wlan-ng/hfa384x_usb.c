

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/wireless.h>
#include <linux/netdevice.h>
#include <linux/timer.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <asm/byteorder.h>
#include <linux/bitops.h>
#include <linux/list.h>
#include <linux/usb.h>
#include <linux/byteorder/generic.h>

#define SUBMIT_URB(u, f)  usb_submit_urb(u, f)

#include "p80211types.h"
#include "p80211hdr.h"
#include "p80211mgmt.h"
#include "p80211conv.h"
#include "p80211msg.h"
#include "p80211netdev.h"
#include "p80211req.h"
#include "p80211metadef.h"
#include "p80211metastruct.h"
#include "hfa384x.h"
#include "prism2mgmt.h"

enum cmd_mode {
	DOWAIT = 0,
	DOASYNC
};

#define THROTTLE_JIFFIES	(HZ/8)
#define URB_ASYNC_UNLINK 0
#define USB_QUEUE_BULK 0

#define ROUNDUP64(a) (((a)+63)&~63)

#ifdef DEBUG_USB
static void dbprint_urb(struct urb *urb);
#endif

static void
hfa384x_int_rxmonitor(wlandevice_t *wlandev, hfa384x_usb_rxfrm_t *rxfrm);

static void hfa384x_usb_defer(struct work_struct *data);

static int submit_rx_urb(hfa384x_t *hw, gfp_t flags);

static int submit_tx_urb(hfa384x_t *hw, struct urb *tx_urb, gfp_t flags);

/*---------------------------------------------------*/
/* Callbacks */
static void hfa384x_usbout_callback(struct urb *urb);
static void hfa384x_ctlxout_callback(struct urb *urb);
static void hfa384x_usbin_callback(struct urb *urb);

static void
hfa384x_usbin_txcompl(wlandevice_t *wlandev, hfa384x_usbin_t * usbin);

static void hfa384x_usbin_rx(wlandevice_t *wlandev, struct sk_buff *skb);

static void hfa384x_usbin_info(wlandevice_t *wlandev, hfa384x_usbin_t * usbin);

static void
hfa384x_usbout_tx(wlandevice_t *wlandev, hfa384x_usbout_t *usbout);

static void hfa384x_usbin_ctlx(hfa384x_t *hw, hfa384x_usbin_t *usbin,
			       int urb_status);

/*---------------------------------------------------*/
/* Functions to support the prism2 usb command queue */

static void hfa384x_usbctlxq_run(hfa384x_t *hw);

static void hfa384x_usbctlx_reqtimerfn(unsigned long data);

static void hfa384x_usbctlx_resptimerfn(unsigned long data);

static void hfa384x_usb_throttlefn(unsigned long data);

static void hfa384x_usbctlx_completion_task(unsigned long data);

static void hfa384x_usbctlx_reaper_task(unsigned long data);

static int hfa384x_usbctlx_submit(hfa384x_t *hw, hfa384x_usbctlx_t *ctlx);

static void unlocked_usbctlx_complete(hfa384x_t *hw, hfa384x_usbctlx_t *ctlx);

struct usbctlx_completor {
	int (*complete) (struct usbctlx_completor *);
};

static int
hfa384x_usbctlx_complete_sync(hfa384x_t *hw,
			      hfa384x_usbctlx_t *ctlx,
			      struct usbctlx_completor *completor);

static int
unlocked_usbctlx_cancel_async(hfa384x_t *hw, hfa384x_usbctlx_t *ctlx);

static void hfa384x_cb_status(hfa384x_t *hw, const hfa384x_usbctlx_t *ctlx);

static void hfa384x_cb_rrid(hfa384x_t *hw, const hfa384x_usbctlx_t *ctlx);

static int
usbctlx_get_status(const hfa384x_usb_cmdresp_t *cmdresp,
		   hfa384x_cmdresult_t *result);

static void
usbctlx_get_rridresult(const hfa384x_usb_rridresp_t *rridresp,
		       hfa384x_rridresult_t *result);

/*---------------------------------------------------*/
/* Low level req/resp CTLX formatters and submitters */
static int
hfa384x_docmd(hfa384x_t *hw,
	      enum cmd_mode mode,
	      hfa384x_metacmd_t *cmd,
	      ctlx_cmdcb_t cmdcb, ctlx_usercb_t usercb, void *usercb_data);

static int
hfa384x_dorrid(hfa384x_t *hw,
	       enum cmd_mode mode,
	       u16 rid,
	       void *riddata,
	       unsigned int riddatalen,
	       ctlx_cmdcb_t cmdcb, ctlx_usercb_t usercb, void *usercb_data);

static int
hfa384x_dowrid(hfa384x_t *hw,
	       enum cmd_mode mode,
	       u16 rid,
	       void *riddata,
	       unsigned int riddatalen,
	       ctlx_cmdcb_t cmdcb, ctlx_usercb_t usercb, void *usercb_data);

static int
hfa384x_dormem(hfa384x_t *hw,
	       enum cmd_mode mode,
	       u16 page,
	       u16 offset,
	       void *data,
	       unsigned int len,
	       ctlx_cmdcb_t cmdcb, ctlx_usercb_t usercb, void *usercb_data);

static int
hfa384x_dowmem(hfa384x_t *hw,
	       enum cmd_mode mode,
	       u16 page,
	       u16 offset,
	       void *data,
	       unsigned int len,
	       ctlx_cmdcb_t cmdcb, ctlx_usercb_t usercb, void *usercb_data);

static int hfa384x_isgood_pdrcode(u16 pdrcode);

static inline const char *ctlxstr(CTLX_STATE s)
{
	static const char *ctlx_str[] = {
		"Initial state",
		"Complete",
		"Request failed",
		"Request pending",
		"Request packet submitted",
		"Request packet completed",
		"Response packet completed"
	};

	return ctlx_str[s];
};

static inline hfa384x_usbctlx_t *get_active_ctlx(hfa384x_t * hw)
{
	return list_entry(hw->ctlxq.active.next, hfa384x_usbctlx_t, list);
}

#ifdef DEBUG_USB
void dbprint_urb(struct urb *urb)
{
	pr_debug("urb->pipe=0x%08x\n", urb->pipe);
	pr_debug("urb->status=0x%08x\n", urb->status);
	pr_debug("urb->transfer_flags=0x%08x\n", urb->transfer_flags);
	pr_debug("urb->transfer_buffer=0x%08x\n",
		 (unsigned int)urb->transfer_buffer);
	pr_debug("urb->transfer_buffer_length=0x%08x\n",
		 urb->transfer_buffer_length);
	pr_debug("urb->actual_length=0x%08x\n", urb->actual_length);
	pr_debug("urb->bandwidth=0x%08x\n", urb->bandwidth);
	pr_debug("urb->setup_packet(ctl)=0x%08x\n",
		 (unsigned int)urb->setup_packet);
	pr_debug("urb->start_frame(iso/irq)=0x%08x\n", urb->start_frame);
	pr_debug("urb->interval(irq)=0x%08x\n", urb->interval);
	pr_debug("urb->error_count(iso)=0x%08x\n", urb->error_count);
	pr_debug("urb->timeout=0x%08x\n", urb->timeout);
	pr_debug("urb->context=0x%08x\n", (unsigned int)urb->context);
	pr_debug("urb->complete=0x%08x\n", (unsigned int)urb->complete);
}
#endif

static int submit_rx_urb(hfa384x_t *hw, gfp_t memflags)
{
	struct sk_buff *skb;
	int result;

	skb = dev_alloc_skb(sizeof(hfa384x_usbin_t));
	if (skb == NULL) {
		result = -ENOMEM;
		goto done;
	}

	/* Post the IN urb */
	usb_fill_bulk_urb(&hw->rx_urb, hw->usb,
			  hw->endp_in,
			  skb->data, sizeof(hfa384x_usbin_t),
			  hfa384x_usbin_callback, hw->wlandev);

	hw->rx_urb_skb = skb;

	result = -ENOLINK;
	if (!hw->wlandev->hwremoved &&
			!test_bit(WORK_RX_HALT, &hw->usb_flags)) {
		result = SUBMIT_URB(&hw->rx_urb, memflags);

		/* Check whether we need to reset the RX pipe */
		if (result == -EPIPE) {
			printk(KERN_WARNING
			       "%s rx pipe stalled: requesting reset\n",
			       hw->wlandev->netdev->name);
			if (!test_and_set_bit(WORK_RX_HALT, &hw->usb_flags))
				schedule_work(&hw->usb_work);
		}
	}

	/* Don't leak memory if anything should go wrong */
	if (result != 0) {
		dev_kfree_skb(skb);
		hw->rx_urb_skb = NULL;
	}

done:
	return result;
}

static int submit_tx_urb(hfa384x_t *hw, struct urb *tx_urb, gfp_t memflags)
{
	struct net_device *netdev = hw->wlandev->netdev;
	int result;

	result = -ENOLINK;
	if (netif_running(netdev)) {

		if (!hw->wlandev->hwremoved
		    && !test_bit(WORK_TX_HALT, &hw->usb_flags)) {
			result = SUBMIT_URB(tx_urb, memflags);

			/* Test whether we need to reset the TX pipe */
			if (result == -EPIPE) {
				printk(KERN_WARNING
				       "%s tx pipe stalled: requesting reset\n",
				       netdev->name);
				set_bit(WORK_TX_HALT, &hw->usb_flags);
				schedule_work(&hw->usb_work);
			} else if (result == 0) {
				netif_stop_queue(netdev);
			}
		}
	}

	return result;
}

static void hfa384x_usb_defer(struct work_struct *data)
{
	hfa384x_t *hw = container_of(data, struct hfa384x, usb_work);
	struct net_device *netdev = hw->wlandev->netdev;

	/* Don't bother trying to reset anything if the plug
	 * has been pulled ...
	 */
	if (hw->wlandev->hwremoved)
		return;

	/* Reception has stopped: try to reset the input pipe */
	if (test_bit(WORK_RX_HALT, &hw->usb_flags)) {
		int ret;

		usb_kill_urb(&hw->rx_urb); /* Cannot be holding spinlock! */

		ret = usb_clear_halt(hw->usb, hw->endp_in);
		if (ret != 0) {
			printk(KERN_ERR
			       "Failed to clear rx pipe for %s: err=%d\n",
			       netdev->name, ret);
		} else {
			printk(KERN_INFO "%s rx pipe reset complete.\n",
			       netdev->name);
			clear_bit(WORK_RX_HALT, &hw->usb_flags);
			set_bit(WORK_RX_RESUME, &hw->usb_flags);
		}
	}

	/* Resume receiving data back from the device. */
	if (test_bit(WORK_RX_RESUME, &hw->usb_flags)) {
		int ret;

		ret = submit_rx_urb(hw, GFP_KERNEL);
		if (ret != 0) {
			printk(KERN_ERR
			       "Failed to resume %s rx pipe.\n", netdev->name);
		} else {
			clear_bit(WORK_RX_RESUME, &hw->usb_flags);
		}
	}

	/* Transmission has stopped: try to reset the output pipe */
	if (test_bit(WORK_TX_HALT, &hw->usb_flags)) {
		int ret;

		usb_kill_urb(&hw->tx_urb);
		ret = usb_clear_halt(hw->usb, hw->endp_out);
		if (ret != 0) {
			printk(KERN_ERR
			       "Failed to clear tx pipe for %s: err=%d\n",
			       netdev->name, ret);
		} else {
			printk(KERN_INFO "%s tx pipe reset complete.\n",
			       netdev->name);
			clear_bit(WORK_TX_HALT, &hw->usb_flags);
			set_bit(WORK_TX_RESUME, &hw->usb_flags);

			/* Stopping the BULK-OUT pipe also blocked
			 * us from sending any more CTLX URBs, so
			 * we need to re-run our queue ...
			 */
			hfa384x_usbctlxq_run(hw);
		}
	}

	/* Resume transmitting. */
	if (test_and_clear_bit(WORK_TX_RESUME, &hw->usb_flags))
		netif_wake_queue(hw->wlandev->netdev);
}

void hfa384x_create(hfa384x_t *hw, struct usb_device *usb)
{
	memset(hw, 0, sizeof(hfa384x_t));
	hw->usb = usb;

	/* set up the endpoints */
	hw->endp_in = usb_rcvbulkpipe(usb, 1);
	hw->endp_out = usb_sndbulkpipe(usb, 2);

	/* Set up the waitq */
	init_waitqueue_head(&hw->cmdq);

	/* Initialize the command queue */
	spin_lock_init(&hw->ctlxq.lock);
	INIT_LIST_HEAD(&hw->ctlxq.pending);
	INIT_LIST_HEAD(&hw->ctlxq.active);
	INIT_LIST_HEAD(&hw->ctlxq.completing);
	INIT_LIST_HEAD(&hw->ctlxq.reapable);

	/* Initialize the authentication queue */
	skb_queue_head_init(&hw->authq);

	tasklet_init(&hw->reaper_bh,
		     hfa384x_usbctlx_reaper_task, (unsigned long)hw);
	tasklet_init(&hw->completion_bh,
		     hfa384x_usbctlx_completion_task, (unsigned long)hw);
	INIT_WORK(&hw->link_bh, prism2sta_processing_defer);
	INIT_WORK(&hw->usb_work, hfa384x_usb_defer);

	init_timer(&hw->throttle);
	hw->throttle.function = hfa384x_usb_throttlefn;
	hw->throttle.data = (unsigned long)hw;

	init_timer(&hw->resptimer);
	hw->resptimer.function = hfa384x_usbctlx_resptimerfn;
	hw->resptimer.data = (unsigned long)hw;

	init_timer(&hw->reqtimer);
	hw->reqtimer.function = hfa384x_usbctlx_reqtimerfn;
	hw->reqtimer.data = (unsigned long)hw;

	usb_init_urb(&hw->rx_urb);
	usb_init_urb(&hw->tx_urb);
	usb_init_urb(&hw->ctlx_urb);

	hw->link_status = HFA384x_LINK_NOTCONNECTED;
	hw->state = HFA384x_STATE_INIT;

	INIT_WORK(&hw->commsqual_bh, prism2sta_commsqual_defer);
	init_timer(&hw->commsqual_timer);
	hw->commsqual_timer.data = (unsigned long)hw;
	hw->commsqual_timer.function = prism2sta_commsqual_timer;
}

void hfa384x_destroy(hfa384x_t *hw)
{
	struct sk_buff *skb;

	if (hw->state == HFA384x_STATE_RUNNING)
		hfa384x_drvr_stop(hw);
	hw->state = HFA384x_STATE_PREINIT;

	if (hw->scanresults) {
		kfree(hw->scanresults);
		hw->scanresults = NULL;
	}

	/* Now to clean out the auth queue */
	while ((skb = skb_dequeue(&hw->authq)))
		dev_kfree_skb(skb);
}

static hfa384x_usbctlx_t *usbctlx_alloc(void)
{
	hfa384x_usbctlx_t *ctlx;

	ctlx = kmalloc(sizeof(*ctlx), in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);
	if (ctlx != NULL) {
		memset(ctlx, 0, sizeof(*ctlx));
		init_completion(&ctlx->done);
	}

	return ctlx;
}

static int
usbctlx_get_status(const hfa384x_usb_cmdresp_t *cmdresp,
		   hfa384x_cmdresult_t *result)
{
	result->status = le16_to_cpu(cmdresp->status);
	result->resp0 = le16_to_cpu(cmdresp->resp0);
	result->resp1 = le16_to_cpu(cmdresp->resp1);
	result->resp2 = le16_to_cpu(cmdresp->resp2);

	pr_debug("cmdresult:status=0x%04x "
		 "resp0=0x%04x resp1=0x%04x resp2=0x%04x\n",
		 result->status, result->resp0, result->resp1, result->resp2);

	return result->status & HFA384x_STATUS_RESULT;
}

static void
usbctlx_get_rridresult(const hfa384x_usb_rridresp_t *rridresp,
		       hfa384x_rridresult_t *result)
{
	result->rid = le16_to_cpu(rridresp->rid);
	result->riddata = rridresp->data;
	result->riddata_len = ((le16_to_cpu(rridresp->frmlen) - 1) * 2);

}

struct usbctlx_cmd_completor {
	struct usbctlx_completor head;

	const hfa384x_usb_cmdresp_t *cmdresp;
	hfa384x_cmdresult_t *result;
};

static inline int usbctlx_cmd_completor_fn(struct usbctlx_completor *head)
{
	struct usbctlx_cmd_completor *complete;

	complete = (struct usbctlx_cmd_completor *) head;
	return usbctlx_get_status(complete->cmdresp, complete->result);
}

static inline struct usbctlx_completor *init_cmd_completor(
						struct usbctlx_cmd_completor
							*completor,
						const hfa384x_usb_cmdresp_t
							*cmdresp,
						hfa384x_cmdresult_t *result)
{
	completor->head.complete = usbctlx_cmd_completor_fn;
	completor->cmdresp = cmdresp;
	completor->result = result;
	return &(completor->head);
}

struct usbctlx_rrid_completor {
	struct usbctlx_completor head;

	const hfa384x_usb_rridresp_t *rridresp;
	void *riddata;
	unsigned int riddatalen;
};

static int usbctlx_rrid_completor_fn(struct usbctlx_completor *head)
{
	struct usbctlx_rrid_completor *complete;
	hfa384x_rridresult_t rridresult;

	complete = (struct usbctlx_rrid_completor *) head;
	usbctlx_get_rridresult(complete->rridresp, &rridresult);

	/* Validate the length, note body len calculation in bytes */
	if (rridresult.riddata_len != complete->riddatalen) {
		printk(KERN_WARNING
		       "RID len mismatch, rid=0x%04x hlen=%d fwlen=%d\n",
		       rridresult.rid,
		       complete->riddatalen, rridresult.riddata_len);
		return -ENODATA;
	}

	memcpy(complete->riddata, rridresult.riddata, complete->riddatalen);
	return 0;
}

static inline struct usbctlx_completor *init_rrid_completor(
						struct usbctlx_rrid_completor
							*completor,
						const hfa384x_usb_rridresp_t
							*rridresp,
						void *riddata,
						unsigned int riddatalen)
{
	completor->head.complete = usbctlx_rrid_completor_fn;
	completor->rridresp = rridresp;
	completor->riddata = riddata;
	completor->riddatalen = riddatalen;
	return &(completor->head);
}

typedef struct usbctlx_cmd_completor usbctlx_wrid_completor_t;
#define init_wrid_completor  init_cmd_completor

typedef struct usbctlx_cmd_completor usbctlx_wmem_completor_t;
#define init_wmem_completor  init_cmd_completor

struct usbctlx_rmem_completor {
	struct usbctlx_completor head;

	const hfa384x_usb_rmemresp_t *rmemresp;
	void *data;
	unsigned int len;
};
typedef struct usbctlx_rmem_completor usbctlx_rmem_completor_t;

static int usbctlx_rmem_completor_fn(struct usbctlx_completor *head)
{
	usbctlx_rmem_completor_t *complete = (usbctlx_rmem_completor_t *) head;

	pr_debug("rmemresp:len=%d\n", complete->rmemresp->frmlen);
	memcpy(complete->data, complete->rmemresp->data, complete->len);
	return 0;
}

static inline struct usbctlx_completor *init_rmem_completor(
						usbctlx_rmem_completor_t
							*completor,
						hfa384x_usb_rmemresp_t
							*rmemresp,
						void *data,
						unsigned int len)
{
	completor->head.complete = usbctlx_rmem_completor_fn;
	completor->rmemresp = rmemresp;
	completor->data = data;
	completor->len = len;
	return &(completor->head);
}

static void hfa384x_cb_status(hfa384x_t *hw, const hfa384x_usbctlx_t *ctlx)
{
	if (ctlx->usercb != NULL) {
		hfa384x_cmdresult_t cmdresult;

		if (ctlx->state != CTLX_COMPLETE) {
			memset(&cmdresult, 0, sizeof(cmdresult));
			cmdresult.status =
			    HFA384x_STATUS_RESULT_SET(HFA384x_CMD_ERR);
		} else {
			usbctlx_get_status(&ctlx->inbuf.cmdresp, &cmdresult);
		}

		ctlx->usercb(hw, &cmdresult, ctlx->usercb_data);
	}
}

static void hfa384x_cb_rrid(hfa384x_t *hw, const hfa384x_usbctlx_t *ctlx)
{
	if (ctlx->usercb != NULL) {
		hfa384x_rridresult_t rridresult;

		if (ctlx->state != CTLX_COMPLETE) {
			memset(&rridresult, 0, sizeof(rridresult));
			rridresult.rid = le16_to_cpu(ctlx->outbuf.rridreq.rid);
		} else {
			usbctlx_get_rridresult(&ctlx->inbuf.rridresp,
					       &rridresult);
		}

		ctlx->usercb(hw, &rridresult, ctlx->usercb_data);
	}
}

static inline int hfa384x_docmd_wait(hfa384x_t *hw, hfa384x_metacmd_t *cmd)
{
	return hfa384x_docmd(hw, DOWAIT, cmd, NULL, NULL, NULL);
}

static inline int
hfa384x_docmd_async(hfa384x_t *hw,
		    hfa384x_metacmd_t *cmd,
		    ctlx_cmdcb_t cmdcb, ctlx_usercb_t usercb, void *usercb_data)
{
	return hfa384x_docmd(hw, DOASYNC, cmd, cmdcb, usercb, usercb_data);
}

static inline int
hfa384x_dorrid_wait(hfa384x_t *hw, u16 rid, void *riddata,
		    unsigned int riddatalen)
{
	return hfa384x_dorrid(hw, DOWAIT,
			      rid, riddata, riddatalen, NULL, NULL, NULL);
}

static inline int
hfa384x_dorrid_async(hfa384x_t *hw,
		     u16 rid, void *riddata, unsigned int riddatalen,
		     ctlx_cmdcb_t cmdcb,
		     ctlx_usercb_t usercb, void *usercb_data)
{
	return hfa384x_dorrid(hw, DOASYNC,
			      rid, riddata, riddatalen,
			      cmdcb, usercb, usercb_data);
}

static inline int
hfa384x_dowrid_wait(hfa384x_t *hw, u16 rid, void *riddata,
		    unsigned int riddatalen)
{
	return hfa384x_dowrid(hw, DOWAIT,
			      rid, riddata, riddatalen, NULL, NULL, NULL);
}

static inline int
hfa384x_dowrid_async(hfa384x_t *hw,
		     u16 rid, void *riddata, unsigned int riddatalen,
		     ctlx_cmdcb_t cmdcb,
		     ctlx_usercb_t usercb, void *usercb_data)
{
	return hfa384x_dowrid(hw, DOASYNC,
			      rid, riddata, riddatalen,
			      cmdcb, usercb, usercb_data);
}

static inline int
hfa384x_dormem_wait(hfa384x_t *hw,
		    u16 page, u16 offset, void *data, unsigned int len)
{
	return hfa384x_dormem(hw, DOWAIT,
			      page, offset, data, len, NULL, NULL, NULL);
}

static inline int
hfa384x_dormem_async(hfa384x_t *hw,
		     u16 page, u16 offset, void *data, unsigned int len,
		     ctlx_cmdcb_t cmdcb,
		     ctlx_usercb_t usercb, void *usercb_data)
{
	return hfa384x_dormem(hw, DOASYNC,
			      page, offset, data, len,
			      cmdcb, usercb, usercb_data);
}

static inline int
hfa384x_dowmem_wait(hfa384x_t *hw,
		    u16 page, u16 offset, void *data, unsigned int len)
{
	return hfa384x_dowmem(hw, DOWAIT,
			      page, offset, data, len, NULL, NULL, NULL);
}

static inline int
hfa384x_dowmem_async(hfa384x_t *hw,
		     u16 page,
		     u16 offset,
		     void *data,
		     unsigned int len,
		     ctlx_cmdcb_t cmdcb,
		     ctlx_usercb_t usercb, void *usercb_data)
{
	return hfa384x_dowmem(hw, DOASYNC,
			      page, offset, data, len,
			      cmdcb, usercb, usercb_data);
}

int hfa384x_cmd_initialize(hfa384x_t *hw)
{
	int result = 0;
	int i;
	hfa384x_metacmd_t cmd;

	cmd.cmd = HFA384x_CMDCODE_INIT;
	cmd.parm0 = 0;
	cmd.parm1 = 0;
	cmd.parm2 = 0;

	result = hfa384x_docmd_wait(hw, &cmd);

	pr_debug("cmdresp.init: "
		 "status=0x%04x, resp0=0x%04x, "
		 "resp1=0x%04x, resp2=0x%04x\n",
		 cmd.result.status,
		 cmd.result.resp0, cmd.result.resp1, cmd.result.resp2);
	if (result == 0) {
		for (i = 0; i < HFA384x_NUMPORTS_MAX; i++)
			hw->port_enabled[i] = 0;
	}

	hw->link_status = HFA384x_LINK_NOTCONNECTED;

	return result;
}

int hfa384x_cmd_disable(hfa384x_t *hw, u16 macport)
{
	int result = 0;
	hfa384x_metacmd_t cmd;

	cmd.cmd = HFA384x_CMD_CMDCODE_SET(HFA384x_CMDCODE_DISABLE) |
	    HFA384x_CMD_MACPORT_SET(macport);
	cmd.parm0 = 0;
	cmd.parm1 = 0;
	cmd.parm2 = 0;

	result = hfa384x_docmd_wait(hw, &cmd);

	return result;
}

int hfa384x_cmd_enable(hfa384x_t *hw, u16 macport)
{
	int result = 0;
	hfa384x_metacmd_t cmd;

	cmd.cmd = HFA384x_CMD_CMDCODE_SET(HFA384x_CMDCODE_ENABLE) |
	    HFA384x_CMD_MACPORT_SET(macport);
	cmd.parm0 = 0;
	cmd.parm1 = 0;
	cmd.parm2 = 0;

	result = hfa384x_docmd_wait(hw, &cmd);

	return result;
}

int hfa384x_cmd_monitor(hfa384x_t *hw, u16 enable)
{
	int result = 0;
	hfa384x_metacmd_t cmd;

	cmd.cmd = HFA384x_CMD_CMDCODE_SET(HFA384x_CMDCODE_MONITOR) |
	    HFA384x_CMD_AINFO_SET(enable);
	cmd.parm0 = 0;
	cmd.parm1 = 0;
	cmd.parm2 = 0;

	result = hfa384x_docmd_wait(hw, &cmd);

	return result;
}

int hfa384x_cmd_download(hfa384x_t *hw, u16 mode, u16 lowaddr,
			 u16 highaddr, u16 codelen)
{
	int result = 0;
	hfa384x_metacmd_t cmd;

	pr_debug("mode=%d, lowaddr=0x%04x, highaddr=0x%04x, codelen=%d\n",
		 mode, lowaddr, highaddr, codelen);

	cmd.cmd = (HFA384x_CMD_CMDCODE_SET(HFA384x_CMDCODE_DOWNLD) |
		   HFA384x_CMD_PROGMODE_SET(mode));

	cmd.parm0 = lowaddr;
	cmd.parm1 = highaddr;
	cmd.parm2 = codelen;

	result = hfa384x_docmd_wait(hw, &cmd);

	return result;
}

int hfa384x_corereset(hfa384x_t *hw, int holdtime, int settletime, int genesis)
{
	int result = 0;

	result = usb_reset_device(hw->usb);
	if (result < 0) {
		printk(KERN_ERR "usb_reset_device() failed, result=%d.\n",
		       result);
	}

	return result;
}

static int hfa384x_usbctlx_complete_sync(hfa384x_t *hw,
					 hfa384x_usbctlx_t *ctlx,
					 struct usbctlx_completor *completor)
{
	unsigned long flags;
	int result;

	result = wait_for_completion_interruptible(&ctlx->done);

	spin_lock_irqsave(&hw->ctlxq.lock, flags);

	/*
	 * We can only handle the CTLX if the USB disconnect
	 * function has not run yet ...
	 */
cleanup:
	if (hw->wlandev->hwremoved) {
		spin_unlock_irqrestore(&hw->ctlxq.lock, flags);
		result = -ENODEV;
	} else if (result != 0) {
		int runqueue = 0;

		/*
		 * We were probably interrupted, so delete
		 * this CTLX asynchronously, kill the timers
		 * and the URB, and then start the next
		 * pending CTLX.
		 *
		 * NOTE: We can only delete the timers and
		 *       the URB if this CTLX is active.
		 */
		if (ctlx == get_active_ctlx(hw)) {
			spin_unlock_irqrestore(&hw->ctlxq.lock, flags);

			del_singleshot_timer_sync(&hw->reqtimer);
			del_singleshot_timer_sync(&hw->resptimer);
			hw->req_timer_done = 1;
			hw->resp_timer_done = 1;
			usb_kill_urb(&hw->ctlx_urb);

			spin_lock_irqsave(&hw->ctlxq.lock, flags);

			runqueue = 1;

			/*
			 * This scenario is so unlikely that I'm
			 * happy with a grubby "goto" solution ...
			 */
			if (hw->wlandev->hwremoved)
				goto cleanup;
		}

		/*
		 * The completion task will send this CTLX
		 * to the reaper the next time it runs. We
		 * are no longer in a hurry.
		 */
		ctlx->reapable = 1;
		ctlx->state = CTLX_REQ_FAILED;
		list_move_tail(&ctlx->list, &hw->ctlxq.completing);

		spin_unlock_irqrestore(&hw->ctlxq.lock, flags);

		if (runqueue)
			hfa384x_usbctlxq_run(hw);
	} else {
		if (ctlx->state == CTLX_COMPLETE) {
			result = completor->complete(completor);
		} else {
			printk(KERN_WARNING "CTLX[%d] error: state(%s)\n",
			       le16_to_cpu(ctlx->outbuf.type),
			       ctlxstr(ctlx->state));
			result = -EIO;
		}

		list_del(&ctlx->list);
		spin_unlock_irqrestore(&hw->ctlxq.lock, flags);
		kfree(ctlx);
	}

	return result;
}

static int
hfa384x_docmd(hfa384x_t *hw,
	      enum cmd_mode mode,
	      hfa384x_metacmd_t *cmd,
	      ctlx_cmdcb_t cmdcb, ctlx_usercb_t usercb, void *usercb_data)
{
	int result;
	hfa384x_usbctlx_t *ctlx;

	ctlx = usbctlx_alloc();
	if (ctlx == NULL) {
		result = -ENOMEM;
		goto done;
	}

	/* Initialize the command */
	ctlx->outbuf.cmdreq.type = cpu_to_le16(HFA384x_USB_CMDREQ);
	ctlx->outbuf.cmdreq.cmd = cpu_to_le16(cmd->cmd);
	ctlx->outbuf.cmdreq.parm0 = cpu_to_le16(cmd->parm0);
	ctlx->outbuf.cmdreq.parm1 = cpu_to_le16(cmd->parm1);
	ctlx->outbuf.cmdreq.parm2 = cpu_to_le16(cmd->parm2);

	ctlx->outbufsize = sizeof(ctlx->outbuf.cmdreq);

	pr_debug("cmdreq: cmd=0x%04x "
		 "parm0=0x%04x parm1=0x%04x parm2=0x%04x\n",
		 cmd->cmd, cmd->parm0, cmd->parm1, cmd->parm2);

	ctlx->reapable = mode;
	ctlx->cmdcb = cmdcb;
	ctlx->usercb = usercb;
	ctlx->usercb_data = usercb_data;

	result = hfa384x_usbctlx_submit(hw, ctlx);
	if (result != 0) {
		kfree(ctlx);
	} else if (mode == DOWAIT) {
		struct usbctlx_cmd_completor completor;

		result =
		    hfa384x_usbctlx_complete_sync(hw, ctlx,
						  init_cmd_completor(&completor,
								     &ctlx->
								     inbuf.
								     cmdresp,
								     &cmd->
								     result));
	}

done:
	return result;
}

static int
hfa384x_dorrid(hfa384x_t *hw,
	       enum cmd_mode mode,
	       u16 rid,
	       void *riddata,
	       unsigned int riddatalen,
	       ctlx_cmdcb_t cmdcb, ctlx_usercb_t usercb, void *usercb_data)
{
	int result;
	hfa384x_usbctlx_t *ctlx;

	ctlx = usbctlx_alloc();
	if (ctlx == NULL) {
		result = -ENOMEM;
		goto done;
	}

	/* Initialize the command */
	ctlx->outbuf.rridreq.type = cpu_to_le16(HFA384x_USB_RRIDREQ);
	ctlx->outbuf.rridreq.frmlen =
	    cpu_to_le16(sizeof(ctlx->outbuf.rridreq.rid));
	ctlx->outbuf.rridreq.rid = cpu_to_le16(rid);

	ctlx->outbufsize = sizeof(ctlx->outbuf.rridreq);

	ctlx->reapable = mode;
	ctlx->cmdcb = cmdcb;
	ctlx->usercb = usercb;
	ctlx->usercb_data = usercb_data;

	/* Submit the CTLX */
	result = hfa384x_usbctlx_submit(hw, ctlx);
	if (result != 0) {
		kfree(ctlx);
	} else if (mode == DOWAIT) {
		struct usbctlx_rrid_completor completor;

		result =
		    hfa384x_usbctlx_complete_sync(hw, ctlx,
						  init_rrid_completor
						  (&completor,
						   &ctlx->inbuf.rridresp,
						   riddata, riddatalen));
	}

done:
	return result;
}

static int
hfa384x_dowrid(hfa384x_t *hw,
	       enum cmd_mode mode,
	       u16 rid,
	       void *riddata,
	       unsigned int riddatalen,
	       ctlx_cmdcb_t cmdcb, ctlx_usercb_t usercb, void *usercb_data)
{
	int result;
	hfa384x_usbctlx_t *ctlx;

	ctlx = usbctlx_alloc();
	if (ctlx == NULL) {
		result = -ENOMEM;
		goto done;
	}

	/* Initialize the command */
	ctlx->outbuf.wridreq.type = cpu_to_le16(HFA384x_USB_WRIDREQ);
	ctlx->outbuf.wridreq.frmlen = cpu_to_le16((sizeof
						   (ctlx->outbuf.wridreq.rid) +
						   riddatalen + 1) / 2);
	ctlx->outbuf.wridreq.rid = cpu_to_le16(rid);
	memcpy(ctlx->outbuf.wridreq.data, riddata, riddatalen);

	ctlx->outbufsize = sizeof(ctlx->outbuf.wridreq.type) +
	    sizeof(ctlx->outbuf.wridreq.frmlen) +
	    sizeof(ctlx->outbuf.wridreq.rid) + riddatalen;

	ctlx->reapable = mode;
	ctlx->cmdcb = cmdcb;
	ctlx->usercb = usercb;
	ctlx->usercb_data = usercb_data;

	/* Submit the CTLX */
	result = hfa384x_usbctlx_submit(hw, ctlx);
	if (result != 0) {
		kfree(ctlx);
	} else if (mode == DOWAIT) {
		usbctlx_wrid_completor_t completor;
		hfa384x_cmdresult_t wridresult;

		result = hfa384x_usbctlx_complete_sync(hw,
						       ctlx,
						       init_wrid_completor
						       (&completor,
							&ctlx->inbuf.wridresp,
							&wridresult));
	}

done:
	return result;
}

static int
hfa384x_dormem(hfa384x_t *hw,
	       enum cmd_mode mode,
	       u16 page,
	       u16 offset,
	       void *data,
	       unsigned int len,
	       ctlx_cmdcb_t cmdcb, ctlx_usercb_t usercb, void *usercb_data)
{
	int result;
	hfa384x_usbctlx_t *ctlx;

	ctlx = usbctlx_alloc();
	if (ctlx == NULL) {
		result = -ENOMEM;
		goto done;
	}

	/* Initialize the command */
	ctlx->outbuf.rmemreq.type = cpu_to_le16(HFA384x_USB_RMEMREQ);
	ctlx->outbuf.rmemreq.frmlen =
	    cpu_to_le16(sizeof(ctlx->outbuf.rmemreq.offset) +
			sizeof(ctlx->outbuf.rmemreq.page) + len);
	ctlx->outbuf.rmemreq.offset = cpu_to_le16(offset);
	ctlx->outbuf.rmemreq.page = cpu_to_le16(page);

	ctlx->outbufsize = sizeof(ctlx->outbuf.rmemreq);

	pr_debug("type=0x%04x frmlen=%d offset=0x%04x page=0x%04x\n",
		 ctlx->outbuf.rmemreq.type,
		 ctlx->outbuf.rmemreq.frmlen,
		 ctlx->outbuf.rmemreq.offset, ctlx->outbuf.rmemreq.page);

	pr_debug("pktsize=%zd\n", ROUNDUP64(sizeof(ctlx->outbuf.rmemreq)));

	ctlx->reapable = mode;
	ctlx->cmdcb = cmdcb;
	ctlx->usercb = usercb;
	ctlx->usercb_data = usercb_data;

	result = hfa384x_usbctlx_submit(hw, ctlx);
	if (result != 0) {
		kfree(ctlx);
	} else if (mode == DOWAIT) {
		usbctlx_rmem_completor_t completor;

		result =
		    hfa384x_usbctlx_complete_sync(hw, ctlx,
						  init_rmem_completor
						  (&completor,
						   &ctlx->inbuf.rmemresp, data,
						   len));
	}

done:
	return result;
}

static int
hfa384x_dowmem(hfa384x_t *hw,
	       enum cmd_mode mode,
	       u16 page,
	       u16 offset,
	       void *data,
	       unsigned int len,
	       ctlx_cmdcb_t cmdcb, ctlx_usercb_t usercb, void *usercb_data)
{
	int result;
	hfa384x_usbctlx_t *ctlx;

	pr_debug("page=0x%04x offset=0x%04x len=%d\n", page, offset, len);

	ctlx = usbctlx_alloc();
	if (ctlx == NULL) {
		result = -ENOMEM;
		goto done;
	}

	/* Initialize the command */
	ctlx->outbuf.wmemreq.type = cpu_to_le16(HFA384x_USB_WMEMREQ);
	ctlx->outbuf.wmemreq.frmlen =
	    cpu_to_le16(sizeof(ctlx->outbuf.wmemreq.offset) +
			sizeof(ctlx->outbuf.wmemreq.page) + len);
	ctlx->outbuf.wmemreq.offset = cpu_to_le16(offset);
	ctlx->outbuf.wmemreq.page = cpu_to_le16(page);
	memcpy(ctlx->outbuf.wmemreq.data, data, len);

	ctlx->outbufsize = sizeof(ctlx->outbuf.wmemreq.type) +
	    sizeof(ctlx->outbuf.wmemreq.frmlen) +
	    sizeof(ctlx->outbuf.wmemreq.offset) +
	    sizeof(ctlx->outbuf.wmemreq.page) + len;

	ctlx->reapable = mode;
	ctlx->cmdcb = cmdcb;
	ctlx->usercb = usercb;
	ctlx->usercb_data = usercb_data;

	result = hfa384x_usbctlx_submit(hw, ctlx);
	if (result != 0) {
		kfree(ctlx);
	} else if (mode == DOWAIT) {
		usbctlx_wmem_completor_t completor;
		hfa384x_cmdresult_t wmemresult;

		result = hfa384x_usbctlx_complete_sync(hw,
						       ctlx,
						       init_wmem_completor
						       (&completor,
							&ctlx->inbuf.wmemresp,
							&wmemresult));
	}

done:
	return result;
}

int hfa384x_drvr_commtallies(hfa384x_t *hw)
{
	hfa384x_metacmd_t cmd;

	cmd.cmd = HFA384x_CMDCODE_INQ;
	cmd.parm0 = HFA384x_IT_COMMTALLIES;
	cmd.parm1 = 0;
	cmd.parm2 = 0;

	hfa384x_docmd_async(hw, &cmd, NULL, NULL, NULL);

	return 0;
}

int hfa384x_drvr_disable(hfa384x_t *hw, u16 macport)
{
	int result = 0;

	if ((!hw->isap && macport != 0) ||
	    (hw->isap && !(macport <= HFA384x_PORTID_MAX)) ||
	    !(hw->port_enabled[macport])) {
		result = -EINVAL;
	} else {
		result = hfa384x_cmd_disable(hw, macport);
		if (result == 0)
			hw->port_enabled[macport] = 0;
	}
	return result;
}

int hfa384x_drvr_enable(hfa384x_t *hw, u16 macport)
{
	int result = 0;

	if ((!hw->isap && macport != 0) ||
	    (hw->isap && !(macport <= HFA384x_PORTID_MAX)) ||
	    (hw->port_enabled[macport])) {
		result = -EINVAL;
	} else {
		result = hfa384x_cmd_enable(hw, macport);
		if (result == 0)
			hw->port_enabled[macport] = 1;
	}
	return result;
}

int hfa384x_drvr_flashdl_enable(hfa384x_t *hw)
{
	int result = 0;
	int i;

	/* Check that a port isn't active */
	for (i = 0; i < HFA384x_PORTID_MAX; i++) {
		if (hw->port_enabled[i]) {
			pr_debug("called when port enabled.\n");
			return -EINVAL;
		}
	}

	/* Check that we're not already in a download state */
	if (hw->dlstate != HFA384x_DLSTATE_DISABLED)
		return -EINVAL;

	/* Retrieve the buffer loc&size and timeout */
	result = hfa384x_drvr_getconfig(hw, HFA384x_RID_DOWNLOADBUFFER,
					&(hw->bufinfo), sizeof(hw->bufinfo));
	if (result)
		return result;

	hw->bufinfo.page = le16_to_cpu(hw->bufinfo.page);
	hw->bufinfo.offset = le16_to_cpu(hw->bufinfo.offset);
	hw->bufinfo.len = le16_to_cpu(hw->bufinfo.len);
	result = hfa384x_drvr_getconfig16(hw, HFA384x_RID_MAXLOADTIME,
					  &(hw->dltimeout));
	if (result)
		return result;

	hw->dltimeout = le16_to_cpu(hw->dltimeout);

	pr_debug("flashdl_enable\n");

	hw->dlstate = HFA384x_DLSTATE_FLASHENABLED;

	return result;
}

int hfa384x_drvr_flashdl_disable(hfa384x_t *hw)
{
	/* Check that we're already in the download state */
	if (hw->dlstate != HFA384x_DLSTATE_FLASHENABLED)
		return -EINVAL;

	pr_debug("flashdl_enable\n");

	/* There isn't much we can do at this point, so I don't */
	/*  bother  w/ the return value */
	hfa384x_cmd_download(hw, HFA384x_PROGMODE_DISABLE, 0, 0, 0);
	hw->dlstate = HFA384x_DLSTATE_DISABLED;

	return 0;
}

int hfa384x_drvr_flashdl_write(hfa384x_t *hw, u32 daddr, void *buf, u32 len)
{
	int result = 0;
	u32 dlbufaddr;
	int nburns;
	u32 burnlen;
	u32 burndaddr;
	u16 burnlo;
	u16 burnhi;
	int nwrites;
	u8 *writebuf;
	u16 writepage;
	u16 writeoffset;
	u32 writelen;
	int i;
	int j;

	pr_debug("daddr=0x%08x len=%d\n", daddr, len);

	/* Check that we're in the flash download state */
	if (hw->dlstate != HFA384x_DLSTATE_FLASHENABLED)
		return -EINVAL;

	printk(KERN_INFO "Download %d bytes to flash @0x%06x\n", len, daddr);

	/* Convert to flat address for arithmetic */
	/* NOTE: dlbuffer RID stores the address in AUX format */
	dlbufaddr =
	    HFA384x_ADDR_AUX_MKFLAT(hw->bufinfo.page, hw->bufinfo.offset);
	pr_debug("dlbuf.page=0x%04x dlbuf.offset=0x%04x dlbufaddr=0x%08x\n",
		 hw->bufinfo.page, hw->bufinfo.offset, dlbufaddr);

#if 0
	printk(KERN_WARNING "dlbuf@0x%06lx len=%d to=%d\n", dlbufaddr,
	       hw->bufinfo.len, hw->dltimeout);
#endif
	/* Calculations to determine how many fills of the dlbuffer to do
	 * and how many USB wmemreq's to do for each fill.  At this point
	 * in time, the dlbuffer size and the wmemreq size are the same.
	 * Therefore, nwrites should always be 1.  The extra complexity
	 * here is a hedge against future changes.
	 */

	/* Figure out how many times to do the flash programming */
	nburns = len / hw->bufinfo.len;
	nburns += (len % hw->bufinfo.len) ? 1 : 0;

	/* For each flash program cycle, how many USB wmemreq's are needed? */
	nwrites = hw->bufinfo.len / HFA384x_USB_RWMEM_MAXLEN;
	nwrites += (hw->bufinfo.len % HFA384x_USB_RWMEM_MAXLEN) ? 1 : 0;

	/* For each burn */
	for (i = 0; i < nburns; i++) {
		/* Get the dest address and len */
		burnlen = (len - (hw->bufinfo.len * i)) > hw->bufinfo.len ?
		    hw->bufinfo.len : (len - (hw->bufinfo.len * i));
		burndaddr = daddr + (hw->bufinfo.len * i);
		burnlo = HFA384x_ADDR_CMD_MKOFF(burndaddr);
		burnhi = HFA384x_ADDR_CMD_MKPAGE(burndaddr);

		printk(KERN_INFO "Writing %d bytes to flash @0x%06x\n",
		       burnlen, burndaddr);

		/* Set the download mode */
		result = hfa384x_cmd_download(hw, HFA384x_PROGMODE_NV,
					      burnlo, burnhi, burnlen);
		if (result) {
			printk(KERN_ERR "download(NV,lo=%x,hi=%x,len=%x) "
			       "cmd failed, result=%d. Aborting d/l\n",
			       burnlo, burnhi, burnlen, result);
			goto exit_proc;
		}

		/* copy the data to the flash download buffer */
		for (j = 0; j < nwrites; j++) {
			writebuf = buf +
			    (i * hw->bufinfo.len) +
			    (j * HFA384x_USB_RWMEM_MAXLEN);

			writepage = HFA384x_ADDR_CMD_MKPAGE(dlbufaddr +
						(j * HFA384x_USB_RWMEM_MAXLEN));
			writeoffset = HFA384x_ADDR_CMD_MKOFF(dlbufaddr +
						(j * HFA384x_USB_RWMEM_MAXLEN));

			writelen = burnlen - (j * HFA384x_USB_RWMEM_MAXLEN);
			writelen = writelen > HFA384x_USB_RWMEM_MAXLEN ?
			    HFA384x_USB_RWMEM_MAXLEN : writelen;

			result = hfa384x_dowmem_wait(hw,
						     writepage,
						     writeoffset,
						     writebuf, writelen);
		}

		/* set the download 'write flash' mode */
		result = hfa384x_cmd_download(hw,
					      HFA384x_PROGMODE_NVWRITE,
					      0, 0, 0);
		if (result) {
			printk(KERN_ERR
			       "download(NVWRITE,lo=%x,hi=%x,len=%x) "
			       "cmd failed, result=%d. Aborting d/l\n",
			       burnlo, burnhi, burnlen, result);
			goto exit_proc;
		}

		/* TODO: We really should do a readback and compare. */
	}

exit_proc:

	/* Leave the firmware in the 'post-prog' mode.  flashdl_disable will */
	/*  actually disable programming mode.  Remember, that will cause the */
	/*  the firmware to effectively reset itself. */

	return result;
}

int hfa384x_drvr_getconfig(hfa384x_t *hw, u16 rid, void *buf, u16 len)
{
	int result;

	result = hfa384x_dorrid_wait(hw, rid, buf, len);

	return result;
}

int
hfa384x_drvr_getconfig_async(hfa384x_t *hw,
			     u16 rid, ctlx_usercb_t usercb, void *usercb_data)
{
	return hfa384x_dorrid_async(hw, rid, NULL, 0,
				    hfa384x_cb_rrid, usercb, usercb_data);
}

int
hfa384x_drvr_setconfig_async(hfa384x_t *hw,
			     u16 rid,
			     void *buf,
			     u16 len, ctlx_usercb_t usercb, void *usercb_data)
{
	return hfa384x_dowrid_async(hw, rid, buf, len,
				    hfa384x_cb_status, usercb, usercb_data);
}

int hfa384x_drvr_ramdl_disable(hfa384x_t *hw)
{
	/* Check that we're already in the download state */
	if (hw->dlstate != HFA384x_DLSTATE_RAMENABLED)
		return -EINVAL;

	pr_debug("ramdl_disable()\n");

	/* There isn't much we can do at this point, so I don't */
	/*  bother  w/ the return value */
	hfa384x_cmd_download(hw, HFA384x_PROGMODE_DISABLE, 0, 0, 0);
	hw->dlstate = HFA384x_DLSTATE_DISABLED;

	return 0;
}

int hfa384x_drvr_ramdl_enable(hfa384x_t *hw, u32 exeaddr)
{
	int result = 0;
	u16 lowaddr;
	u16 hiaddr;
	int i;

	/* Check that a port isn't active */
	for (i = 0; i < HFA384x_PORTID_MAX; i++) {
		if (hw->port_enabled[i]) {
			printk(KERN_ERR
			       "Can't download with a macport enabled.\n");
			return -EINVAL;
		}
	}

	/* Check that we're not already in a download state */
	if (hw->dlstate != HFA384x_DLSTATE_DISABLED) {
		printk(KERN_ERR "Download state not disabled.\n");
		return -EINVAL;
	}

	pr_debug("ramdl_enable, exeaddr=0x%08x\n", exeaddr);

	/* Call the download(1,addr) function */
	lowaddr = HFA384x_ADDR_CMD_MKOFF(exeaddr);
	hiaddr = HFA384x_ADDR_CMD_MKPAGE(exeaddr);

	result = hfa384x_cmd_download(hw, HFA384x_PROGMODE_RAM,
				      lowaddr, hiaddr, 0);

	if (result == 0) {
		/* Set the download state */
		hw->dlstate = HFA384x_DLSTATE_RAMENABLED;
	} else {
		pr_debug("cmd_download(0x%04x, 0x%04x) failed, result=%d.\n",
			 lowaddr, hiaddr, result);
	}

	return result;
}

int hfa384x_drvr_ramdl_write(hfa384x_t *hw, u32 daddr, void *buf, u32 len)
{
	int result = 0;
	int nwrites;
	u8 *data = buf;
	int i;
	u32 curraddr;
	u16 currpage;
	u16 curroffset;
	u16 currlen;

	/* Check that we're in the ram download state */
	if (hw->dlstate != HFA384x_DLSTATE_RAMENABLED)
		return -EINVAL;

	printk(KERN_INFO "Writing %d bytes to ram @0x%06x\n", len, daddr);

	/* How many dowmem calls?  */
	nwrites = len / HFA384x_USB_RWMEM_MAXLEN;
	nwrites += len % HFA384x_USB_RWMEM_MAXLEN ? 1 : 0;

	/* Do blocking wmem's */
	for (i = 0; i < nwrites; i++) {
		/* make address args */
		curraddr = daddr + (i * HFA384x_USB_RWMEM_MAXLEN);
		currpage = HFA384x_ADDR_CMD_MKPAGE(curraddr);
		curroffset = HFA384x_ADDR_CMD_MKOFF(curraddr);
		currlen = len - (i * HFA384x_USB_RWMEM_MAXLEN);
		if (currlen > HFA384x_USB_RWMEM_MAXLEN)
			currlen = HFA384x_USB_RWMEM_MAXLEN;

		/* Do blocking ctlx */
		result = hfa384x_dowmem_wait(hw,
					     currpage,
					     curroffset,
					     data +
					     (i * HFA384x_USB_RWMEM_MAXLEN),
					     currlen);

		if (result)
			break;

		/* TODO: We really should have a readback. */
	}

	return result;
}

int hfa384x_drvr_readpda(hfa384x_t *hw, void *buf, unsigned int len)
{
	int result = 0;
	u16 *pda = buf;
	int pdaok = 0;
	int morepdrs = 1;
	int currpdr = 0;	/* word offset of the current pdr */
	size_t i;
	u16 pdrlen;		/* pdr length in bytes, host order */
	u16 pdrcode;		/* pdr code, host order */
	u16 currpage;
	u16 curroffset;
	struct pdaloc {
		u32 cardaddr;
		u16 auxctl;
	} pdaloc[] = {
		{
		HFA3842_PDA_BASE, 0}, {
		HFA3841_PDA_BASE, 0}, {
		HFA3841_PDA_BOGUS_BASE, 0}
	};

	/* Read the pda from each known address.  */
	for (i = 0; i < ARRAY_SIZE(pdaloc); i++) {
		/* Make address */
		currpage = HFA384x_ADDR_CMD_MKPAGE(pdaloc[i].cardaddr);
		curroffset = HFA384x_ADDR_CMD_MKOFF(pdaloc[i].cardaddr);

		/* units of bytes */
		result = hfa384x_dormem_wait(hw, currpage, curroffset, buf,
						len);

		if (result) {
			printk(KERN_WARNING
			       "Read from index %zd failed, continuing\n", i);
			continue;
		}

		/* Test for garbage */
		pdaok = 1;	/* initially assume good */
		morepdrs = 1;
		while (pdaok && morepdrs) {
			pdrlen = le16_to_cpu(pda[currpdr]) * 2;
			pdrcode = le16_to_cpu(pda[currpdr + 1]);
			/* Test the record length */
			if (pdrlen > HFA384x_PDR_LEN_MAX || pdrlen == 0) {
				printk(KERN_ERR "pdrlen invalid=%d\n", pdrlen);
				pdaok = 0;
				break;
			}
			/* Test the code */
			if (!hfa384x_isgood_pdrcode(pdrcode)) {
				printk(KERN_ERR "pdrcode invalid=%d\n",
				       pdrcode);
				pdaok = 0;
				break;
			}
			/* Test for completion */
			if (pdrcode == HFA384x_PDR_END_OF_PDA)
				morepdrs = 0;

			/* Move to the next pdr (if necessary) */
			if (morepdrs) {
				/* note the access to pda[], need words here */
				currpdr += le16_to_cpu(pda[currpdr]) + 1;
			}
		}
		if (pdaok) {
			printk(KERN_INFO
			       "PDA Read from 0x%08x in %s space.\n",
			       pdaloc[i].cardaddr,
			       pdaloc[i].auxctl == 0 ? "EXTDS" :
			       pdaloc[i].auxctl == 1 ? "NV" :
			       pdaloc[i].auxctl == 2 ? "PHY" :
			       pdaloc[i].auxctl == 3 ? "ICSRAM" :
			       "<bogus auxctl>");
			break;
		}
	}
	result = pdaok ? 0 : -ENODATA;

	if (result)
		pr_debug("Failure: pda is not okay\n");

	return result;
}

int hfa384x_drvr_setconfig(hfa384x_t *hw, u16 rid, void *buf, u16 len)
{
	return hfa384x_dowrid_wait(hw, rid, buf, len);
}


int hfa384x_drvr_start(hfa384x_t *hw)
{
	int result, result1, result2;
	u16 status;

	might_sleep();

	/* Clear endpoint stalls - but only do this if the endpoint
	 * is showing a stall status. Some prism2 cards seem to behave
	 * badly if a clear_halt is called when the endpoint is already
	 * ok
	 */
	result =
	    usb_get_status(hw->usb, USB_RECIP_ENDPOINT, hw->endp_in, &status);
	if (result < 0) {
		printk(KERN_ERR "Cannot get bulk in endpoint status.\n");
		goto done;
	}
	if ((status == 1) && usb_clear_halt(hw->usb, hw->endp_in))
		printk(KERN_ERR "Failed to reset bulk in endpoint.\n");

	result =
	    usb_get_status(hw->usb, USB_RECIP_ENDPOINT, hw->endp_out, &status);
	if (result < 0) {
		printk(KERN_ERR "Cannot get bulk out endpoint status.\n");
		goto done;
	}
	if ((status == 1) && usb_clear_halt(hw->usb, hw->endp_out))
		printk(KERN_ERR "Failed to reset bulk out endpoint.\n");

	/* Synchronous unlink, in case we're trying to restart the driver */
	usb_kill_urb(&hw->rx_urb);

	/* Post the IN urb */
	result = submit_rx_urb(hw, GFP_KERNEL);
	if (result != 0) {
		printk(KERN_ERR
		       "Fatal, failed to submit RX URB, result=%d\n", result);
		goto done;
	}

	/* Call initialize twice, with a 1 second sleep in between.
	 * This is a nasty work-around since many prism2 cards seem to
	 * need time to settle after an init from cold. The second
	 * call to initialize in theory is not necessary - but we call
	 * it anyway as a double insurance policy:
	 * 1) If the first init should fail, the second may well succeed
	 *    and the card can still be used
	 * 2) It helps ensures all is well with the card after the first
	 *    init and settle time.
	 */
	result1 = hfa384x_cmd_initialize(hw);
	msleep(1000);
	result = result2 = hfa384x_cmd_initialize(hw);
	if (result1 != 0) {
		if (result2 != 0) {
			printk(KERN_ERR
				"cmd_initialize() failed on two attempts, results %d and %d\n",
				result1, result2);
			usb_kill_urb(&hw->rx_urb);
			goto done;
		} else {
			pr_debug("First cmd_initialize() failed (result %d),\n",
				 result1);
			pr_debug("but second attempt succeeded. All should be ok\n");
		}
	} else if (result2 != 0) {
		printk(KERN_WARNING "First cmd_initialize() succeeded, but second attempt failed (result=%d)\n",
			result2);
		printk(KERN_WARNING
		       "Most likely the card will be functional\n");
		goto done;
	}

	hw->state = HFA384x_STATE_RUNNING;

done:
	return result;
}

int hfa384x_drvr_stop(hfa384x_t *hw)
{
	int result = 0;
	int i;

	might_sleep();

	/* There's no need for spinlocks here. The USB "disconnect"
	 * function sets this "removed" flag and then calls us.
	 */
	if (!hw->wlandev->hwremoved) {
		/* Call initialize to leave the MAC in its 'reset' state */
		hfa384x_cmd_initialize(hw);

		/* Cancel the rxurb */
		usb_kill_urb(&hw->rx_urb);
	}

	hw->link_status = HFA384x_LINK_NOTCONNECTED;
	hw->state = HFA384x_STATE_INIT;

	del_timer_sync(&hw->commsqual_timer);

	/* Clear all the port status */
	for (i = 0; i < HFA384x_NUMPORTS_MAX; i++)
		hw->port_enabled[i] = 0;

	return result;
}

int hfa384x_drvr_txframe(hfa384x_t *hw, struct sk_buff *skb,
			 p80211_hdr_t *p80211_hdr,
			 p80211_metawep_t *p80211_wep)
{
	int usbpktlen = sizeof(hfa384x_tx_frame_t);
	int result;
	int ret;
	char *ptr;

	if (hw->tx_urb.status == -EINPROGRESS) {
		printk(KERN_WARNING "TX URB already in use\n");
		result = 3;
		goto exit;
	}

	/* Build Tx frame structure */
	/* Set up the control field */
	memset(&hw->txbuff.txfrm.desc, 0, sizeof(hw->txbuff.txfrm.desc));

	/* Setup the usb type field */
	hw->txbuff.type = cpu_to_le16(HFA384x_USB_TXFRM);

	/* Set up the sw_support field to identify this frame */
	hw->txbuff.txfrm.desc.sw_support = 0x0123;

/* #define DOEXC  SLP -- doboth breaks horribly under load, doexc less so. */
#if defined(DOBOTH)
	hw->txbuff.txfrm.desc.tx_control =
	    HFA384x_TX_MACPORT_SET(0) | HFA384x_TX_STRUCTYPE_SET(1) |
	    HFA384x_TX_TXEX_SET(1) | HFA384x_TX_TXOK_SET(1);
#elif defined(DOEXC)
	hw->txbuff.txfrm.desc.tx_control =
	    HFA384x_TX_MACPORT_SET(0) | HFA384x_TX_STRUCTYPE_SET(1) |
	    HFA384x_TX_TXEX_SET(1) | HFA384x_TX_TXOK_SET(0);
#else
	hw->txbuff.txfrm.desc.tx_control =
	    HFA384x_TX_MACPORT_SET(0) | HFA384x_TX_STRUCTYPE_SET(1) |
	    HFA384x_TX_TXEX_SET(0) | HFA384x_TX_TXOK_SET(0);
#endif
	hw->txbuff.txfrm.desc.tx_control =
	    cpu_to_le16(hw->txbuff.txfrm.desc.tx_control);

	/* copy the header over to the txdesc */
	memcpy(&(hw->txbuff.txfrm.desc.frame_control), p80211_hdr,
	       sizeof(p80211_hdr_t));

	/* if we're using host WEP, increase size by IV+ICV */
	if (p80211_wep->data) {
		hw->txbuff.txfrm.desc.data_len = cpu_to_le16(skb->len + 8);
		usbpktlen += 8;
	} else {
		hw->txbuff.txfrm.desc.data_len = cpu_to_le16(skb->len);
	}

	usbpktlen += skb->len;

	/* copy over the WEP IV if we are using host WEP */
	ptr = hw->txbuff.txfrm.data;
	if (p80211_wep->data) {
		memcpy(ptr, p80211_wep->iv, sizeof(p80211_wep->iv));
		ptr += sizeof(p80211_wep->iv);
		memcpy(ptr, p80211_wep->data, skb->len);
	} else {
		memcpy(ptr, skb->data, skb->len);
	}
	/* copy over the packet data */
	ptr += skb->len;

	/* copy over the WEP ICV if we are using host WEP */
	if (p80211_wep->data)
		memcpy(ptr, p80211_wep->icv, sizeof(p80211_wep->icv));

	/* Send the USB packet */
	usb_fill_bulk_urb(&(hw->tx_urb), hw->usb,
			  hw->endp_out,
			  &(hw->txbuff), ROUNDUP64(usbpktlen),
			  hfa384x_usbout_callback, hw->wlandev);
	hw->tx_urb.transfer_flags |= USB_QUEUE_BULK;

	result = 1;
	ret = submit_tx_urb(hw, &hw->tx_urb, GFP_ATOMIC);
	if (ret != 0) {
		printk(KERN_ERR "submit_tx_urb() failed, error=%d\n", ret);
		result = 3;
	}

exit:
	return result;
}

void hfa384x_tx_timeout(wlandevice_t *wlandev)
{
	hfa384x_t *hw = wlandev->priv;
	unsigned long flags;

	spin_lock_irqsave(&hw->ctlxq.lock, flags);

	if (!hw->wlandev->hwremoved &&
	    /* Note the bitwise OR, not the logical OR. */
	    (!test_and_set_bit(WORK_TX_HALT, &hw->usb_flags) |
	     !test_and_set_bit(WORK_RX_HALT, &hw->usb_flags))) {
		schedule_work(&hw->usb_work);
	}

	spin_unlock_irqrestore(&hw->ctlxq.lock, flags);
}

static void hfa384x_usbctlx_reaper_task(unsigned long data)
{
	hfa384x_t *hw = (hfa384x_t *) data;
	struct list_head *entry;
	struct list_head *temp;
	unsigned long flags;

	spin_lock_irqsave(&hw->ctlxq.lock, flags);

	/* This list is guaranteed to be empty if someone
	 * has unplugged the adapter.
	 */
	list_for_each_safe(entry, temp, &hw->ctlxq.reapable) {
		hfa384x_usbctlx_t *ctlx;

		ctlx = list_entry(entry, hfa384x_usbctlx_t, list);
		list_del(&ctlx->list);
		kfree(ctlx);
	}

	spin_unlock_irqrestore(&hw->ctlxq.lock, flags);

}

static void hfa384x_usbctlx_completion_task(unsigned long data)
{
	hfa384x_t *hw = (hfa384x_t *) data;
	struct list_head *entry;
	struct list_head *temp;
	unsigned long flags;

	int reap = 0;

	spin_lock_irqsave(&hw->ctlxq.lock, flags);

	/* This list is guaranteed to be empty if someone
	 * has unplugged the adapter ...
	 */
	list_for_each_safe(entry, temp, &hw->ctlxq.completing) {
		hfa384x_usbctlx_t *ctlx;

		ctlx = list_entry(entry, hfa384x_usbctlx_t, list);

		/* Call the completion function that this
		 * command was assigned, assuming it has one.
		 */
		if (ctlx->cmdcb != NULL) {
			spin_unlock_irqrestore(&hw->ctlxq.lock, flags);
			ctlx->cmdcb(hw, ctlx);
			spin_lock_irqsave(&hw->ctlxq.lock, flags);

			/* Make sure we don't try and complete
			 * this CTLX more than once!
			 */
			ctlx->cmdcb = NULL;

			/* Did someone yank the adapter out
			 * while our list was (briefly) unlocked?
			 */
			if (hw->wlandev->hwremoved) {
				reap = 0;
				break;
			}
		}

		/*
		 * "Reapable" CTLXs are ones which don't have any
		 * threads waiting for them to die. Hence they must
		 * be delivered to The Reaper!
		 */
		if (ctlx->reapable) {
			/* Move the CTLX off the "completing" list (hopefully)
			 * on to the "reapable" list where the reaper task
			 * can find it. And "reapable" means that this CTLX
			 * isn't sitting on a wait-queue somewhere.
			 */
			list_move_tail(&ctlx->list, &hw->ctlxq.reapable);
			reap = 1;
		}

		complete(&ctlx->done);
	}
	spin_unlock_irqrestore(&hw->ctlxq.lock, flags);

	if (reap)
		tasklet_schedule(&hw->reaper_bh);
}

static int unlocked_usbctlx_cancel_async(hfa384x_t *hw,
					 hfa384x_usbctlx_t *ctlx)
{
	int ret;

	/*
	 * Try to delete the URB containing our request packet.
	 * If we succeed, then its completion handler will be
	 * called with a status of -ECONNRESET.
	 */
	hw->ctlx_urb.transfer_flags |= URB_ASYNC_UNLINK;
	ret = usb_unlink_urb(&hw->ctlx_urb);

	if (ret != -EINPROGRESS) {
		/*
		 * The OUT URB had either already completed
		 * or was still in the pending queue, so the
		 * URB's completion function will not be called.
		 * We will have to complete the CTLX ourselves.
		 */
		ctlx->state = CTLX_REQ_FAILED;
		unlocked_usbctlx_complete(hw, ctlx);
		ret = 0;
	}

	return ret;
}

static void unlocked_usbctlx_complete(hfa384x_t *hw, hfa384x_usbctlx_t *ctlx)
{
	/* Timers have been stopped, and ctlx should be in
	 * a terminal state. Retire it from the "active"
	 * queue.
	 */
	list_move_tail(&ctlx->list, &hw->ctlxq.completing);
	tasklet_schedule(&hw->completion_bh);

	switch (ctlx->state) {
	case CTLX_COMPLETE:
	case CTLX_REQ_FAILED:
		/* This are the correct terminating states. */
		break;

	default:
		printk(KERN_ERR "CTLX[%d] not in a terminating state(%s)\n",
		       le16_to_cpu(ctlx->outbuf.type), ctlxstr(ctlx->state));
		break;
	}			/* switch */
}

static void hfa384x_usbctlxq_run(hfa384x_t *hw)
{
	unsigned long flags;

	/* acquire lock */
	spin_lock_irqsave(&hw->ctlxq.lock, flags);

	/* Only one active CTLX at any one time, because there's no
	 * other (reliable) way to match the response URB to the
	 * correct CTLX.
	 *
	 * Don't touch any of these CTLXs if the hardware
	 * has been removed or the USB subsystem is stalled.
	 */
	if (!list_empty(&hw->ctlxq.active) ||
	    test_bit(WORK_TX_HALT, &hw->usb_flags) || hw->wlandev->hwremoved)
		goto unlock;

	while (!list_empty(&hw->ctlxq.pending)) {
		hfa384x_usbctlx_t *head;
		int result;

		/* This is the first pending command */
		head = list_entry(hw->ctlxq.pending.next,
				  hfa384x_usbctlx_t, list);

		/* We need to split this off to avoid a race condition */
		list_move_tail(&head->list, &hw->ctlxq.active);

		/* Fill the out packet */
		usb_fill_bulk_urb(&(hw->ctlx_urb), hw->usb,
				  hw->endp_out,
				  &(head->outbuf), ROUNDUP64(head->outbufsize),
				  hfa384x_ctlxout_callback, hw);
		hw->ctlx_urb.transfer_flags |= USB_QUEUE_BULK;

		/* Now submit the URB and update the CTLX's state */
		result = SUBMIT_URB(&hw->ctlx_urb, GFP_ATOMIC);
		if (result == 0) {
			/* This CTLX is now running on the active queue */
			head->state = CTLX_REQ_SUBMITTED;

			/* Start the OUT wait timer */
			hw->req_timer_done = 0;
			hw->reqtimer.expires = jiffies + HZ;
			add_timer(&hw->reqtimer);

			/* Start the IN wait timer */
			hw->resp_timer_done = 0;
			hw->resptimer.expires = jiffies + 2 * HZ;
			add_timer(&hw->resptimer);

			break;
		}

		if (result == -EPIPE) {
			/* The OUT pipe needs resetting, so put
			 * this CTLX back in the "pending" queue
			 * and schedule a reset ...
			 */
			printk(KERN_WARNING
			       "%s tx pipe stalled: requesting reset\n",
			       hw->wlandev->netdev->name);
			list_move(&head->list, &hw->ctlxq.pending);
			set_bit(WORK_TX_HALT, &hw->usb_flags);
			schedule_work(&hw->usb_work);
			break;
		}

		if (result == -ESHUTDOWN) {
			printk(KERN_WARNING "%s urb shutdown!\n",
			       hw->wlandev->netdev->name);
			break;
		}

		printk(KERN_ERR "Failed to submit CTLX[%d]: error=%d\n",
		       le16_to_cpu(head->outbuf.type), result);
		unlocked_usbctlx_complete(hw, head);
	}			/* while */

unlock:
	spin_unlock_irqrestore(&hw->ctlxq.lock, flags);
}

static void hfa384x_usbin_callback(struct urb *urb)
{
	wlandevice_t *wlandev = urb->context;
	hfa384x_t *hw;
	hfa384x_usbin_t *usbin = (hfa384x_usbin_t *) urb->transfer_buffer;
	struct sk_buff *skb = NULL;
	int result;
	int urb_status;
	u16 type;

	enum USBIN_ACTION {
		HANDLE,
		RESUBMIT,
		ABORT
	} action;

	if (!wlandev || !wlandev->netdev || wlandev->hwremoved)
		goto exit;

	hw = wlandev->priv;
	if (!hw)
		goto exit;

	skb = hw->rx_urb_skb;
	BUG_ON(!skb || (skb->data != urb->transfer_buffer));

	hw->rx_urb_skb = NULL;

	/* Check for error conditions within the URB */
	switch (urb->status) {
	case 0:
		action = HANDLE;

		/* Check for short packet */
		if (urb->actual_length == 0) {
			++(wlandev->linux_stats.rx_errors);
			++(wlandev->linux_stats.rx_length_errors);
			action = RESUBMIT;
		}
		break;

	case -EPIPE:
		printk(KERN_WARNING "%s rx pipe stalled: requesting reset\n",
		       wlandev->netdev->name);
		if (!test_and_set_bit(WORK_RX_HALT, &hw->usb_flags))
			schedule_work(&hw->usb_work);
		++(wlandev->linux_stats.rx_errors);
		action = ABORT;
		break;

	case -EILSEQ:
	case -ETIMEDOUT:
	case -EPROTO:
		if (!test_and_set_bit(THROTTLE_RX, &hw->usb_flags) &&
		    !timer_pending(&hw->throttle)) {
			mod_timer(&hw->throttle, jiffies + THROTTLE_JIFFIES);
		}
		++(wlandev->linux_stats.rx_errors);
		action = ABORT;
		break;

	case -EOVERFLOW:
		++(wlandev->linux_stats.rx_over_errors);
		action = RESUBMIT;
		break;

	case -ENODEV:
	case -ESHUTDOWN:
		pr_debug("status=%d, device removed.\n", urb->status);
		action = ABORT;
		break;

	case -ENOENT:
	case -ECONNRESET:
		pr_debug("status=%d, urb explicitly unlinked.\n", urb->status);
		action = ABORT;
		break;

	default:
		pr_debug("urb status=%d, transfer flags=0x%x\n",
			 urb->status, urb->transfer_flags);
		++(wlandev->linux_stats.rx_errors);
		action = RESUBMIT;
		break;
	}

	urb_status = urb->status;

	if (action != ABORT) {
		/* Repost the RX URB */
		result = submit_rx_urb(hw, GFP_ATOMIC);

		if (result != 0) {
			printk(KERN_ERR
			       "Fatal, failed to resubmit rx_urb. error=%d\n",
			       result);
		}
	}

	/* Handle any USB-IN packet */
	/* Note: the check of the sw_support field, the type field doesn't
	 *       have bit 12 set like the docs suggest.
	 */
	type = le16_to_cpu(usbin->type);
	if (HFA384x_USB_ISRXFRM(type)) {
		if (action == HANDLE) {
			if (usbin->txfrm.desc.sw_support == 0x0123) {
				hfa384x_usbin_txcompl(wlandev, usbin);
			} else {
				skb_put(skb, sizeof(*usbin));
				hfa384x_usbin_rx(wlandev, skb);
				skb = NULL;
			}
		}
		goto exit;
	}
	if (HFA384x_USB_ISTXFRM(type)) {
		if (action == HANDLE)
			hfa384x_usbin_txcompl(wlandev, usbin);
		goto exit;
	}
	switch (type) {
	case HFA384x_USB_INFOFRM:
		if (action == ABORT)
			goto exit;
		if (action == HANDLE)
			hfa384x_usbin_info(wlandev, usbin);
		break;

	case HFA384x_USB_CMDRESP:
	case HFA384x_USB_WRIDRESP:
	case HFA384x_USB_RRIDRESP:
	case HFA384x_USB_WMEMRESP:
	case HFA384x_USB_RMEMRESP:
		/* ALWAYS, ALWAYS, ALWAYS handle this CTLX!!!! */
		hfa384x_usbin_ctlx(hw, usbin, urb_status);
		break;

	case HFA384x_USB_BUFAVAIL:
		pr_debug("Received BUFAVAIL packet, frmlen=%d\n",
			 usbin->bufavail.frmlen);
		break;

	case HFA384x_USB_ERROR:
		pr_debug("Received USB_ERROR packet, errortype=%d\n",
			 usbin->usberror.errortype);
		break;

	default:
		pr_debug("Unrecognized USBIN packet, type=%x, status=%d\n",
			 usbin->type, urb_status);
		break;
	}			/* switch */

exit:

	if (skb)
		dev_kfree_skb(skb);
}

static void hfa384x_usbin_ctlx(hfa384x_t *hw, hfa384x_usbin_t *usbin,
			       int urb_status)
{
	hfa384x_usbctlx_t *ctlx;
	int run_queue = 0;
	unsigned long flags;

retry:
	spin_lock_irqsave(&hw->ctlxq.lock, flags);

	/* There can be only one CTLX on the active queue
	 * at any one time, and this is the CTLX that the
	 * timers are waiting for.
	 */
	if (list_empty(&hw->ctlxq.active))
		goto unlock;

	/* Remove the "response timeout". It's possible that
	 * we are already too late, and that the timeout is
	 * already running. And that's just too bad for us,
	 * because we could lose our CTLX from the active
	 * queue here ...
	 */
	if (del_timer(&hw->resptimer) == 0) {
		if (hw->resp_timer_done == 0) {
			spin_unlock_irqrestore(&hw->ctlxq.lock, flags);
			goto retry;
		}
	} else {
		hw->resp_timer_done = 1;
	}

	ctlx = get_active_ctlx(hw);

	if (urb_status != 0) {
		/*
		 * Bad CTLX, so get rid of it. But we only
		 * remove it from the active queue if we're no
		 * longer expecting the OUT URB to complete.
		 */
		if (unlocked_usbctlx_cancel_async(hw, ctlx) == 0)
			run_queue = 1;
	} else {
		const u16 intype = (usbin->type & ~cpu_to_le16(0x8000));

		/*
		 * Check that our message is what we're expecting ...
		 */
		if (ctlx->outbuf.type != intype) {
			printk(KERN_WARNING
			       "Expected IN[%d], received IN[%d] - ignored.\n",
			       le16_to_cpu(ctlx->outbuf.type),
			       le16_to_cpu(intype));
			goto unlock;
		}

		/* This URB has succeeded, so grab the data ... */
		memcpy(&ctlx->inbuf, usbin, sizeof(ctlx->inbuf));

		switch (ctlx->state) {
		case CTLX_REQ_SUBMITTED:
			/*
			 * We have received our response URB before
			 * our request has been acknowledged. Odd,
			 * but our OUT URB is still alive...
			 */
			pr_debug("Causality violation: please reboot Universe\n");
			ctlx->state = CTLX_RESP_COMPLETE;
			break;

		case CTLX_REQ_COMPLETE:
			/*
			 * This is the usual path: our request
			 * has already been acknowledged, and
			 * now we have received the reply too.
			 */
			ctlx->state = CTLX_COMPLETE;
			unlocked_usbctlx_complete(hw, ctlx);
			run_queue = 1;
			break;

		default:
			/*
			 * Throw this CTLX away ...
			 */
			printk(KERN_ERR
			       "Matched IN URB, CTLX[%d] in invalid state(%s)."
			       " Discarded.\n",
			       le16_to_cpu(ctlx->outbuf.type),
			       ctlxstr(ctlx->state));
			if (unlocked_usbctlx_cancel_async(hw, ctlx) == 0)
				run_queue = 1;
			break;
		}		/* switch */
	}

unlock:
	spin_unlock_irqrestore(&hw->ctlxq.lock, flags);

	if (run_queue)
		hfa384x_usbctlxq_run(hw);
}

static void hfa384x_usbin_txcompl(wlandevice_t *wlandev,
				  hfa384x_usbin_t *usbin)
{
	u16 status;

	status = le16_to_cpu(usbin->type); /* yeah I know it says type... */

	/* Was there an error? */
	if (HFA384x_TXSTATUS_ISERROR(status))
		prism2sta_ev_txexc(wlandev, status);
	else
		prism2sta_ev_tx(wlandev, status);
}

static void hfa384x_usbin_rx(wlandevice_t *wlandev, struct sk_buff *skb)
{
	hfa384x_usbin_t *usbin = (hfa384x_usbin_t *) skb->data;
	hfa384x_t *hw = wlandev->priv;
	int hdrlen;
	p80211_rxmeta_t *rxmeta;
	u16 data_len;
	u16 fc;

	/* Byte order convert once up front. */
	usbin->rxfrm.desc.status = le16_to_cpu(usbin->rxfrm.desc.status);
	usbin->rxfrm.desc.time = le32_to_cpu(usbin->rxfrm.desc.time);

	/* Now handle frame based on port# */
	switch (HFA384x_RXSTATUS_MACPORT_GET(usbin->rxfrm.desc.status)) {
	case 0:
		fc = le16_to_cpu(usbin->rxfrm.desc.frame_control);

		/* If exclude and we receive an unencrypted, drop it */
		if ((wlandev->hostwep & HOSTWEP_EXCLUDEUNENCRYPTED) &&
		    !WLAN_GET_FC_ISWEP(fc)) {
			goto done;
		}

		data_len = le16_to_cpu(usbin->rxfrm.desc.data_len);

		/* How much header data do we have? */
		hdrlen = p80211_headerlen(fc);

		/* Pull off the descriptor */
		skb_pull(skb, sizeof(hfa384x_rx_frame_t));

		/* Now shunt the header block up against the data block
		 * with an "overlapping" copy
		 */
		memmove(skb_push(skb, hdrlen),
			&usbin->rxfrm.desc.frame_control, hdrlen);

		skb->dev = wlandev->netdev;
		skb->dev->last_rx = jiffies;

		/* And set the frame length properly */
		skb_trim(skb, data_len + hdrlen);

		/* The prism2 series does not return the CRC */
		memset(skb_put(skb, WLAN_CRC_LEN), 0xff, WLAN_CRC_LEN);

		skb_reset_mac_header(skb);

		/* Attach the rxmeta, set some stuff */
		p80211skb_rxmeta_attach(wlandev, skb);
		rxmeta = P80211SKB_RXMETA(skb);
		rxmeta->mactime = usbin->rxfrm.desc.time;
		rxmeta->rxrate = usbin->rxfrm.desc.rate;
		rxmeta->signal = usbin->rxfrm.desc.signal - hw->dbmadjust;
		rxmeta->noise = usbin->rxfrm.desc.silence - hw->dbmadjust;

		prism2sta_ev_rx(wlandev, skb);

		break;

	case 7:
		if (!HFA384x_RXSTATUS_ISFCSERR(usbin->rxfrm.desc.status)) {
			/* Copy to wlansnif skb */
			hfa384x_int_rxmonitor(wlandev, &usbin->rxfrm);
			dev_kfree_skb(skb);
		} else {
			pr_debug("Received monitor frame: FCSerr set\n");
		}
		break;

	default:
		printk(KERN_WARNING "Received frame on unsupported port=%d\n",
		       HFA384x_RXSTATUS_MACPORT_GET(usbin->rxfrm.desc.status));
		goto done;
		break;
	}

done:
	return;
}

static void hfa384x_int_rxmonitor(wlandevice_t *wlandev,
				  hfa384x_usb_rxfrm_t *rxfrm)
{
	hfa384x_rx_frame_t *rxdesc = &(rxfrm->desc);
	unsigned int hdrlen = 0;
	unsigned int datalen = 0;
	unsigned int skblen = 0;
	u8 *datap;
	u16 fc;
	struct sk_buff *skb;
	hfa384x_t *hw = wlandev->priv;

	/* Remember the status, time, and data_len fields are in host order */
	/* Figure out how big the frame is */
	fc = le16_to_cpu(rxdesc->frame_control);
	hdrlen = p80211_headerlen(fc);
	datalen = le16_to_cpu(rxdesc->data_len);

	/* Allocate an ind message+framesize skb */
	skblen = sizeof(p80211_caphdr_t) + hdrlen + datalen + WLAN_CRC_LEN;

	/* sanity check the length */
	if (skblen >
	    (sizeof(p80211_caphdr_t) +
	     WLAN_HDR_A4_LEN + WLAN_DATA_MAXLEN + WLAN_CRC_LEN)) {
		pr_debug("overlen frm: len=%zd\n",
			 skblen - sizeof(p80211_caphdr_t));
	}

	skb = dev_alloc_skb(skblen);
	if (skb == NULL) {
		printk(KERN_ERR
		       "alloc_skb failed trying to allocate %d bytes\n",
		       skblen);
		return;
	}

	/* only prepend the prism header if in the right mode */
	if ((wlandev->netdev->type == ARPHRD_IEEE80211_PRISM) &&
	    (hw->sniffhdr != 0)) {
		p80211_caphdr_t *caphdr;
		/* The NEW header format! */
		datap = skb_put(skb, sizeof(p80211_caphdr_t));
		caphdr = (p80211_caphdr_t *) datap;

		caphdr->version = htonl(P80211CAPTURE_VERSION);
		caphdr->length = htonl(sizeof(p80211_caphdr_t));
		caphdr->mactime = __cpu_to_be64(rxdesc->time) * 1000;
		caphdr->hosttime = __cpu_to_be64(jiffies);
		caphdr->phytype = htonl(4);	/* dss_dot11_b */
		caphdr->channel = htonl(hw->sniff_channel);
		caphdr->datarate = htonl(rxdesc->rate);
		caphdr->antenna = htonl(0);	/* unknown */
		caphdr->priority = htonl(0);	/* unknown */
		caphdr->ssi_type = htonl(3);	/* rssi_raw */
		caphdr->ssi_signal = htonl(rxdesc->signal);
		caphdr->ssi_noise = htonl(rxdesc->silence);
		caphdr->preamble = htonl(0);	/* unknown */
		caphdr->encoding = htonl(1);	/* cck */
	}

	/* Copy the 802.11 header to the skb
	   (ctl frames may be less than a full header) */
	datap = skb_put(skb, hdrlen);
	memcpy(datap, &(rxdesc->frame_control), hdrlen);

	/* If any, copy the data from the card to the skb */
	if (datalen > 0) {
		datap = skb_put(skb, datalen);
		memcpy(datap, rxfrm->data, datalen);

		/* check for unencrypted stuff if WEP bit set. */
		if (*(datap - hdrlen + 1) & 0x40)	/* wep set */
			if ((*(datap) == 0xaa) && (*(datap + 1) == 0xaa))
				/* clear wep; it's the 802.2 header! */
				*(datap - hdrlen + 1) &= 0xbf;
	}

	if (hw->sniff_fcs) {
		/* Set the FCS */
		datap = skb_put(skb, WLAN_CRC_LEN);
		memset(datap, 0xff, WLAN_CRC_LEN);
	}

	/* pass it back up */
	prism2sta_ev_rx(wlandev, skb);

	return;
}

static void hfa384x_usbin_info(wlandevice_t *wlandev, hfa384x_usbin_t *usbin)
{
	usbin->infofrm.info.framelen =
	    le16_to_cpu(usbin->infofrm.info.framelen);
	prism2sta_ev_info(wlandev, &usbin->infofrm.info);
}

static void hfa384x_usbout_callback(struct urb *urb)
{
	wlandevice_t *wlandev = urb->context;
	hfa384x_usbout_t *usbout = urb->transfer_buffer;

#ifdef DEBUG_USB
	dbprint_urb(urb);
#endif

	if (wlandev && wlandev->netdev) {

		switch (urb->status) {
		case 0:
			hfa384x_usbout_tx(wlandev, usbout);
			break;

		case -EPIPE:
			{
				hfa384x_t *hw = wlandev->priv;
				printk(KERN_WARNING
				       "%s tx pipe stalled: requesting reset\n",
				       wlandev->netdev->name);
				if (!test_and_set_bit
				    (WORK_TX_HALT, &hw->usb_flags))
					schedule_work(&hw->usb_work);
				++(wlandev->linux_stats.tx_errors);
				break;
			}

		case -EPROTO:
		case -ETIMEDOUT:
		case -EILSEQ:
			{
				hfa384x_t *hw = wlandev->priv;

				if (!test_and_set_bit
				    (THROTTLE_TX, &hw->usb_flags)
				    && !timer_pending(&hw->throttle)) {
					mod_timer(&hw->throttle,
						  jiffies + THROTTLE_JIFFIES);
				}
				++(wlandev->linux_stats.tx_errors);
				netif_stop_queue(wlandev->netdev);
				break;
			}

		case -ENOENT:
		case -ESHUTDOWN:
			/* Ignorable errors */
			break;

		default:
			printk(KERN_INFO "unknown urb->status=%d\n",
			       urb->status);
			++(wlandev->linux_stats.tx_errors);
			break;
		}		/* switch */
	}
}

static void hfa384x_ctlxout_callback(struct urb *urb)
{
	hfa384x_t *hw = urb->context;
	int delete_resptimer = 0;
	int timer_ok = 1;
	int run_queue = 0;
	hfa384x_usbctlx_t *ctlx;
	unsigned long flags;

	pr_debug("urb->status=%d\n", urb->status);
#ifdef DEBUG_USB
	dbprint_urb(urb);
#endif
	if ((urb->status == -ESHUTDOWN) ||
	    (urb->status == -ENODEV) || (hw == NULL))
		goto done;

retry:
	spin_lock_irqsave(&hw->ctlxq.lock, flags);

	/*
	 * Only one CTLX at a time on the "active" list, and
	 * none at all if we are unplugged. However, we can
	 * rely on the disconnect function to clean everything
	 * up if someone unplugged the adapter.
	 */
	if (list_empty(&hw->ctlxq.active)) {
		spin_unlock_irqrestore(&hw->ctlxq.lock, flags);
		goto done;
	}

	/*
	 * Having something on the "active" queue means
	 * that we have timers to worry about ...
	 */
	if (del_timer(&hw->reqtimer) == 0) {
		if (hw->req_timer_done == 0) {
			/*
			 * This timer was actually running while we
			 * were trying to delete it. Let it terminate
			 * gracefully instead.
			 */
			spin_unlock_irqrestore(&hw->ctlxq.lock, flags);
			goto retry;
		}
	} else {
		hw->req_timer_done = 1;
	}

	ctlx = get_active_ctlx(hw);

	if (urb->status == 0) {
		/* Request portion of a CTLX is successful */
		switch (ctlx->state) {
		case CTLX_REQ_SUBMITTED:
			/* This OUT-ACK received before IN */
			ctlx->state = CTLX_REQ_COMPLETE;
			break;

		case CTLX_RESP_COMPLETE:
			/* IN already received before this OUT-ACK,
			 * so this command must now be complete.
			 */
			ctlx->state = CTLX_COMPLETE;
			unlocked_usbctlx_complete(hw, ctlx);
			run_queue = 1;
			break;

		default:
			/* This is NOT a valid CTLX "success" state! */
			printk(KERN_ERR
				"Illegal CTLX[%d] success state(%s, %d) in OUT URB\n",
				le16_to_cpu(ctlx->outbuf.type),
				ctlxstr(ctlx->state), urb->status);
			break;
		}		/* switch */
	} else {
		/* If the pipe has stalled then we need to reset it */
		if ((urb->status == -EPIPE) &&
		    !test_and_set_bit(WORK_TX_HALT, &hw->usb_flags)) {
			printk(KERN_WARNING
			       "%s tx pipe stalled: requesting reset\n",
			       hw->wlandev->netdev->name);
			schedule_work(&hw->usb_work);
		}

		/* If someone cancels the OUT URB then its status
		 * should be either -ECONNRESET or -ENOENT.
		 */
		ctlx->state = CTLX_REQ_FAILED;
		unlocked_usbctlx_complete(hw, ctlx);
		delete_resptimer = 1;
		run_queue = 1;
	}

delresp:
	if (delete_resptimer) {
		timer_ok = del_timer(&hw->resptimer);
		if (timer_ok != 0)
			hw->resp_timer_done = 1;
	}

	spin_unlock_irqrestore(&hw->ctlxq.lock, flags);

	if (!timer_ok && (hw->resp_timer_done == 0)) {
		spin_lock_irqsave(&hw->ctlxq.lock, flags);
		goto delresp;
	}

	if (run_queue)
		hfa384x_usbctlxq_run(hw);

done:
	;
}

static void hfa384x_usbctlx_reqtimerfn(unsigned long data)
{
	hfa384x_t *hw = (hfa384x_t *) data;
	unsigned long flags;

	spin_lock_irqsave(&hw->ctlxq.lock, flags);

	hw->req_timer_done = 1;

	/* Removing the hardware automatically empties
	 * the active list ...
	 */
	if (!list_empty(&hw->ctlxq.active)) {
		/*
		 * We must ensure that our URB is removed from
		 * the system, if it hasn't already expired.
		 */
		hw->ctlx_urb.transfer_flags |= URB_ASYNC_UNLINK;
		if (usb_unlink_urb(&hw->ctlx_urb) == -EINPROGRESS) {
			hfa384x_usbctlx_t *ctlx = get_active_ctlx(hw);

			ctlx->state = CTLX_REQ_FAILED;

			/* This URB was active, but has now been
			 * cancelled. It will now have a status of
			 * -ECONNRESET in the callback function.
			 *
			 * We are cancelling this CTLX, so we're
			 * not going to need to wait for a response.
			 * The URB's callback function will check
			 * that this timer is truly dead.
			 */
			if (del_timer(&hw->resptimer) != 0)
				hw->resp_timer_done = 1;
		}
	}

	spin_unlock_irqrestore(&hw->ctlxq.lock, flags);
}

static void hfa384x_usbctlx_resptimerfn(unsigned long data)
{
	hfa384x_t *hw = (hfa384x_t *) data;
	unsigned long flags;

	spin_lock_irqsave(&hw->ctlxq.lock, flags);

	hw->resp_timer_done = 1;

	/* The active list will be empty if the
	 * adapter has been unplugged ...
	 */
	if (!list_empty(&hw->ctlxq.active)) {
		hfa384x_usbctlx_t *ctlx = get_active_ctlx(hw);

		if (unlocked_usbctlx_cancel_async(hw, ctlx) == 0) {
			spin_unlock_irqrestore(&hw->ctlxq.lock, flags);
			hfa384x_usbctlxq_run(hw);
			goto done;
		}
	}

	spin_unlock_irqrestore(&hw->ctlxq.lock, flags);

done:
	;

}

static void hfa384x_usb_throttlefn(unsigned long data)
{
	hfa384x_t *hw = (hfa384x_t *) data;
	unsigned long flags;

	spin_lock_irqsave(&hw->ctlxq.lock, flags);

	/*
	 * We need to check BOTH the RX and the TX throttle controls,
	 * so we use the bitwise OR instead of the logical OR.
	 */
	pr_debug("flags=0x%lx\n", hw->usb_flags);
	if (!hw->wlandev->hwremoved &&
	    ((test_and_clear_bit(THROTTLE_RX, &hw->usb_flags) &&
	      !test_and_set_bit(WORK_RX_RESUME, &hw->usb_flags))
	     |
	     (test_and_clear_bit(THROTTLE_TX, &hw->usb_flags) &&
	      !test_and_set_bit(WORK_TX_RESUME, &hw->usb_flags))
	    )) {
		schedule_work(&hw->usb_work);
	}

	spin_unlock_irqrestore(&hw->ctlxq.lock, flags);
}

static int hfa384x_usbctlx_submit(hfa384x_t *hw, hfa384x_usbctlx_t *ctlx)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&hw->ctlxq.lock, flags);

	if (hw->wlandev->hwremoved) {
		spin_unlock_irqrestore(&hw->ctlxq.lock, flags);
		ret = -ENODEV;
	} else {
		ctlx->state = CTLX_PENDING;
		list_add_tail(&ctlx->list, &hw->ctlxq.pending);

		spin_unlock_irqrestore(&hw->ctlxq.lock, flags);
		hfa384x_usbctlxq_run(hw);
		ret = 0;
	}

	return ret;
}

static void hfa384x_usbout_tx(wlandevice_t *wlandev, hfa384x_usbout_t *usbout)
{
	prism2sta_ev_alloc(wlandev);
}

static int hfa384x_isgood_pdrcode(u16 pdrcode)
{
	switch (pdrcode) {
	case HFA384x_PDR_END_OF_PDA:
	case HFA384x_PDR_PCB_PARTNUM:
	case HFA384x_PDR_PDAVER:
	case HFA384x_PDR_NIC_SERIAL:
	case HFA384x_PDR_MKK_MEASUREMENTS:
	case HFA384x_PDR_NIC_RAMSIZE:
	case HFA384x_PDR_MFISUPRANGE:
	case HFA384x_PDR_CFISUPRANGE:
	case HFA384x_PDR_NICID:
	case HFA384x_PDR_MAC_ADDRESS:
	case HFA384x_PDR_REGDOMAIN:
	case HFA384x_PDR_ALLOWED_CHANNEL:
	case HFA384x_PDR_DEFAULT_CHANNEL:
	case HFA384x_PDR_TEMPTYPE:
	case HFA384x_PDR_IFR_SETTING:
	case HFA384x_PDR_RFR_SETTING:
	case HFA384x_PDR_HFA3861_BASELINE:
	case HFA384x_PDR_HFA3861_SHADOW:
	case HFA384x_PDR_HFA3861_IFRF:
	case HFA384x_PDR_HFA3861_CHCALSP:
	case HFA384x_PDR_HFA3861_CHCALI:
	case HFA384x_PDR_3842_NIC_CONFIG:
	case HFA384x_PDR_USB_ID:
	case HFA384x_PDR_PCI_ID:
	case HFA384x_PDR_PCI_IFCONF:
	case HFA384x_PDR_PCI_PMCONF:
	case HFA384x_PDR_RFENRGY:
	case HFA384x_PDR_HFA3861_MANF_TESTSP:
	case HFA384x_PDR_HFA3861_MANF_TESTI:
		/* code is OK */
		return 1;
		break;
	default:
		if (pdrcode < 0x1000) {
			/* code is OK, but we don't know exactly what it is */
			pr_debug("Encountered unknown PDR#=0x%04x, "
				 "assuming it's ok.\n", pdrcode);
			return 1;
		} else {
			/* bad code */
			pr_debug("Encountered unknown PDR#=0x%04x, "
				 "(>=0x1000), assuming it's bad.\n", pdrcode);
			return 0;
		}
		break;
	}
	return 0;		/* avoid compiler warnings */
}
