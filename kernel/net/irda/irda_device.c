

#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/skbuff.h>
#include <linux/capability.h>
#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/if_arp.h>
#include <linux/netdevice.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/kmod.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

#include <asm/ioctls.h>
#include <asm/uaccess.h>
#include <asm/dma.h>
#include <asm/io.h>

#include <net/irda/irda_device.h>
#include <net/irda/irlap.h>
#include <net/irda/timer.h>
#include <net/irda/wrapper.h>

static void __irda_task_delete(struct irda_task *task);

static hashbin_t *dongles = NULL;
static hashbin_t *tasks = NULL;

static void irda_task_timer_expired(void *data);

int __init irda_device_init( void)
{
	dongles = hashbin_new(HB_NOLOCK);
	if (dongles == NULL) {
		IRDA_WARNING("IrDA: Can't allocate dongles hashbin!\n");
		return -ENOMEM;
	}
	spin_lock_init(&dongles->hb_spinlock);

	tasks = hashbin_new(HB_LOCK);
	if (tasks == NULL) {
		IRDA_WARNING("IrDA: Can't allocate tasks hashbin!\n");
		hashbin_delete(dongles, NULL);
		return -ENOMEM;
	}

	/* We no longer initialise the driver ourselves here, we let
	 * the system do it for us... - Jean II */

	return 0;
}

static void leftover_dongle(void *arg)
{
	struct dongle_reg *reg = arg;
	IRDA_WARNING("IrDA: Dongle type %x not unregistered\n",
		     reg->type);
}

void irda_device_cleanup(void)
{
	IRDA_DEBUG(4, "%s()\n", __func__);

	hashbin_delete(tasks, (FREE_FUNC) __irda_task_delete);

	hashbin_delete(dongles, leftover_dongle);
}

void irda_device_set_media_busy(struct net_device *dev, int status)
{
	struct irlap_cb *self;

	IRDA_DEBUG(4, "%s(%s)\n", __func__, status ? "TRUE" : "FALSE");

	self = (struct irlap_cb *) dev->atalk_ptr;

	/* Some drivers may enable the receive interrupt before calling
	 * irlap_open(), or they may disable the receive interrupt
	 * after calling irlap_close().
	 * The IrDA stack is protected from this in irlap_driver_rcv().
	 * However, the driver calls directly the wrapper, that calls
	 * us directly. Make sure we protect ourselves.
	 * Jean II */
	if (!self || self->magic != LAP_MAGIC)
		return;

	if (status) {
		self->media_busy = TRUE;
		if (status == SMALL)
			irlap_start_mbusy_timer(self, SMALLBUSY_TIMEOUT);
		else
			irlap_start_mbusy_timer(self, MEDIABUSY_TIMEOUT);
		IRDA_DEBUG( 4, "Media busy!\n");
	} else {
		self->media_busy = FALSE;
		irlap_stop_mbusy_timer(self);
	}
}
EXPORT_SYMBOL(irda_device_set_media_busy);


int irda_device_is_receiving(struct net_device *dev)
{
	struct if_irda_req req;
	int ret;

	IRDA_DEBUG(2, "%s()\n", __func__);

	if (!dev->netdev_ops->ndo_do_ioctl) {
		IRDA_ERROR("%s: do_ioctl not impl. by device driver\n",
			   __func__);
		return -1;
	}

	ret = (dev->netdev_ops->ndo_do_ioctl)(dev, (struct ifreq *) &req,
					      SIOCGRECEIVING);
	if (ret < 0)
		return ret;

	return req.ifr_receiving;
}

static void __irda_task_delete(struct irda_task *task)
{
	del_timer(&task->timer);

	kfree(task);
}

static void irda_task_delete(struct irda_task *task)
{
	/* Unregister task */
	hashbin_remove(tasks, (long) task, NULL);

	__irda_task_delete(task);
}

static int irda_task_kick(struct irda_task *task)
{
	int finished = TRUE;
	int count = 0;
	int timeout;

	IRDA_DEBUG(2, "%s()\n", __func__);

	IRDA_ASSERT(task != NULL, return -1;);
	IRDA_ASSERT(task->magic == IRDA_TASK_MAGIC, return -1;);

	/* Execute task until it's finished, or askes for a timeout */
	do {
		timeout = task->function(task);
		if (count++ > 100) {
			IRDA_ERROR("%s: error in task handler!\n",
				   __func__);
			irda_task_delete(task);
			return TRUE;
		}
	} while ((timeout == 0) && (task->state != IRDA_TASK_DONE));

	if (timeout < 0) {
		IRDA_ERROR("%s: Error executing task!\n", __func__);
		irda_task_delete(task);
		return TRUE;
	}

	/* Check if we are finished */
	if (task->state == IRDA_TASK_DONE) {
		del_timer(&task->timer);

		/* Do post processing */
		if (task->finished)
			task->finished(task);

		/* Notify parent */
		if (task->parent) {
			/* Check if parent is waiting for us to complete */
			if (task->parent->state == IRDA_TASK_CHILD_WAIT) {
				task->parent->state = IRDA_TASK_CHILD_DONE;

				/* Stop timer now that we are here */
				del_timer(&task->parent->timer);

				/* Kick parent task */
				irda_task_kick(task->parent);
			}
		}
		irda_task_delete(task);
	} else if (timeout > 0) {
		irda_start_timer(&task->timer, timeout, (void *) task,
				 irda_task_timer_expired);
		finished = FALSE;
	} else {
		IRDA_DEBUG(0, "%s(), not finished, and no timeout!\n",
			   __func__);
		finished = FALSE;
	}

	return finished;
}

static void irda_task_timer_expired(void *data)
{
	struct irda_task *task;

	IRDA_DEBUG(2, "%s()\n", __func__);

	task = (struct irda_task *) data;

	irda_task_kick(task);
}

static void irda_device_setup(struct net_device *dev)
{
	dev->hard_header_len = 0;
	dev->addr_len        = LAP_ALEN;

	dev->type            = ARPHRD_IRDA;
	dev->tx_queue_len    = 8; /* Window size + 1 s-frame */

	memset(dev->broadcast, 0xff, LAP_ALEN);

	dev->mtu = 2048;
	dev->flags = IFF_NOARP;
}

struct net_device *alloc_irdadev(int sizeof_priv)
{
	return alloc_netdev(sizeof_priv, "irda%d", irda_device_setup);
}
EXPORT_SYMBOL(alloc_irdadev);

#ifdef CONFIG_ISA_DMA_API
void irda_setup_dma(int channel, dma_addr_t buffer, int count, int mode)
{
	unsigned long flags;

	flags = claim_dma_lock();

	disable_dma(channel);
	clear_dma_ff(channel);
	set_dma_mode(channel, mode);
	set_dma_addr(channel, buffer);
	set_dma_count(channel, count);
	enable_dma(channel);

	release_dma_lock(flags);
}
EXPORT_SYMBOL(irda_setup_dma);
#endif
