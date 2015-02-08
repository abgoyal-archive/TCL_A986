

#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#include "mthca_dev.h"

enum {
	MTHCA_CATAS_POLL_INTERVAL	= 5 * HZ,

	MTHCA_CATAS_TYPE_INTERNAL	= 0,
	MTHCA_CATAS_TYPE_UPLINK		= 3,
	MTHCA_CATAS_TYPE_DDR		= 4,
	MTHCA_CATAS_TYPE_PARITY		= 5,
};

static DEFINE_SPINLOCK(catas_lock);

static LIST_HEAD(catas_list);
static struct workqueue_struct *catas_wq;
static struct work_struct catas_work;

static int catas_reset_disable;
module_param_named(catas_reset_disable, catas_reset_disable, int, 0644);
MODULE_PARM_DESC(catas_reset_disable, "disable reset on catastrophic event if nonzero");

static void catas_reset(struct work_struct *work)
{
	struct mthca_dev *dev, *tmpdev;
	LIST_HEAD(tlist);
	int ret;

	mutex_lock(&mthca_device_mutex);

	spin_lock_irq(&catas_lock);
	list_splice_init(&catas_list, &tlist);
	spin_unlock_irq(&catas_lock);

	list_for_each_entry_safe(dev, tmpdev, &tlist, catas_err.list) {
		struct pci_dev *pdev = dev->pdev;
		ret = __mthca_restart_one(dev->pdev);
		/* 'dev' now is not valid */
		if (ret)
			printk(KERN_ERR "mthca %s: Reset failed (%d)\n",
			       pci_name(pdev), ret);
		else {
			struct mthca_dev *d = pci_get_drvdata(pdev);
			mthca_dbg(d, "Reset succeeded\n");
		}
	}

	mutex_unlock(&mthca_device_mutex);
}

static void handle_catas(struct mthca_dev *dev)
{
	struct ib_event event;
	unsigned long flags;
	const char *type;
	int i;

	event.device = &dev->ib_dev;
	event.event  = IB_EVENT_DEVICE_FATAL;
	event.element.port_num = 0;
	dev->active = false;

	ib_dispatch_event(&event);

	switch (swab32(readl(dev->catas_err.map)) >> 24) {
	case MTHCA_CATAS_TYPE_INTERNAL:
		type = "internal error";
		break;
	case MTHCA_CATAS_TYPE_UPLINK:
		type = "uplink bus error";
		break;
	case MTHCA_CATAS_TYPE_DDR:
		type = "DDR data error";
		break;
	case MTHCA_CATAS_TYPE_PARITY:
		type = "internal parity error";
		break;
	default:
		type = "unknown error";
		break;
	}

	mthca_err(dev, "Catastrophic error detected: %s\n", type);
	for (i = 0; i < dev->catas_err.size; ++i)
		mthca_err(dev, "  buf[%02x]: %08x\n",
			  i, swab32(readl(dev->catas_err.map + i)));

	if (catas_reset_disable)
		return;

	spin_lock_irqsave(&catas_lock, flags);
	list_add(&dev->catas_err.list, &catas_list);
	queue_work(catas_wq, &catas_work);
	spin_unlock_irqrestore(&catas_lock, flags);
}

static void poll_catas(unsigned long dev_ptr)
{
	struct mthca_dev *dev = (struct mthca_dev *) dev_ptr;
	int i;

	for (i = 0; i < dev->catas_err.size; ++i)
		if (readl(dev->catas_err.map + i)) {
			handle_catas(dev);
			return;
		}

	mod_timer(&dev->catas_err.timer,
		  round_jiffies(jiffies + MTHCA_CATAS_POLL_INTERVAL));
}

void mthca_start_catas_poll(struct mthca_dev *dev)
{
	unsigned long addr;

	init_timer(&dev->catas_err.timer);
	dev->catas_err.map  = NULL;

	addr = pci_resource_start(dev->pdev, 0) +
		((pci_resource_len(dev->pdev, 0) - 1) &
		 dev->catas_err.addr);

	dev->catas_err.map = ioremap(addr, dev->catas_err.size * 4);
	if (!dev->catas_err.map) {
		mthca_warn(dev, "couldn't map catastrophic error region "
			   "at 0x%lx/0x%x\n", addr, dev->catas_err.size * 4);
		return;
	}

	dev->catas_err.timer.data     = (unsigned long) dev;
	dev->catas_err.timer.function = poll_catas;
	dev->catas_err.timer.expires  = jiffies + MTHCA_CATAS_POLL_INTERVAL;
	INIT_LIST_HEAD(&dev->catas_err.list);
	add_timer(&dev->catas_err.timer);
}

void mthca_stop_catas_poll(struct mthca_dev *dev)
{
	del_timer_sync(&dev->catas_err.timer);

	if (dev->catas_err.map)
		iounmap(dev->catas_err.map);

	spin_lock_irq(&catas_lock);
	list_del(&dev->catas_err.list);
	spin_unlock_irq(&catas_lock);
}

int __init mthca_catas_init(void)
{
	INIT_WORK(&catas_work, catas_reset);

	catas_wq = create_singlethread_workqueue("mthca_catas");
	if (!catas_wq)
		return -ENOMEM;

	return 0;
}

void mthca_catas_cleanup(void)
{
	destroy_workqueue(catas_wq);
}
