

#include <linux/device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/stat.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <asm/hvconsole.h>
#include <asm/hvcserver.h>
#include <asm/uaccess.h>
#include <asm/vio.h>


#define HVCS_DRIVER_VERSION "1.3.3"

MODULE_AUTHOR("Ryan S. Arnold <rsa@us.ibm.com>");
MODULE_DESCRIPTION("IBM hvcs (Hypervisor Virtual Console Server) Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(HVCS_DRIVER_VERSION);

#define HVCS_CLOSE_WAIT (HZ/100) /* 1/10 of a second */

#define HVCS_DEFAULT_SERVER_ADAPTERS	64

#define HVCS_MAX_SERVER_ADAPTERS	1024

#define HVCS_MINOR_START	0

#define __ALIGNED__	__attribute__((__aligned__(8)))

#define HVCS_BUFF_LEN	16

#define HVCS_MAX_FROM_USER	4096

static struct ktermios hvcs_tty_termios = {
	.c_iflag = IGNBRK | IGNPAR,
	.c_oflag = OPOST,
	.c_cflag = B38400 | CS8 | CREAD | HUPCL,
	.c_cc = INIT_C_CC,
	.c_ispeed = 38400,
	.c_ospeed = 38400
};

static int hvcs_parm_num_devs = -1;
module_param(hvcs_parm_num_devs, int, 0);

static const char hvcs_driver_name[] = "hvcs";
static const char hvcs_device_node[] = "hvcs";
static const char hvcs_driver_string[]
	= "IBM hvcs (Hypervisor Virtual Console Server) Driver";

/* Status of partner info rescan triggered via sysfs. */
static int hvcs_rescan_status;

static struct tty_driver *hvcs_tty_driver;

static int *hvcs_index_list;

static int hvcs_index_count;

static int hvcs_kicked;

static struct task_struct *hvcs_task;

static unsigned long *hvcs_pi_buff;

/* Only allow one hvcs_struct to use the hvcs_pi_buff at a time. */
static DEFINE_SPINLOCK(hvcs_pi_lock);

/* One vty-server per hvcs_struct */
struct hvcs_struct {
	spinlock_t lock;

	/*
	 * This index identifies this hvcs device as the complement to a
	 * specific tty index.
	 */
	unsigned int index;

	struct tty_struct *tty;
	int open_count;

	/*
	 * Used to tell the driver kernel_thread what operations need to take
	 * place upon this hvcs_struct instance.
	 */
	int todo_mask;

	/*
	 * This buffer is required so that when hvcs_write_room() reports that
	 * it can send HVCS_BUFF_LEN characters that it will buffer the full
	 * HVCS_BUFF_LEN characters if need be.  This is essential for opost
	 * writes since they do not do high level buffering and expect to be
	 * able to send what the driver commits to sending buffering
	 * [e.g. tab to space conversions in n_tty.c opost()].
	 */
	char buffer[HVCS_BUFF_LEN];
	int chars_in_buffer;

	/*
	 * Any variable below the kref is valid before a tty is connected and
	 * stays valid after the tty is disconnected.  These shouldn't be
	 * whacked until the koject refcount reaches zero though some entries
	 * may be changed via sysfs initiatives.
	 */
	struct kref kref; /* ref count & hvcs_struct lifetime */
	int connected; /* is the vty-server currently connected to a vty? */
	uint32_t p_unit_address; /* partner unit address */
	uint32_t p_partition_ID; /* partner partition ID */
	char p_location_code[HVCS_CLC_LENGTH + 1]; /* CLC + Null Term */
	struct list_head next; /* list management */
	struct vio_dev *vdev;
};

/* Required to back map a kref to its containing object */
#define from_kref(k) container_of(k, struct hvcs_struct, kref)

static LIST_HEAD(hvcs_structs);
static DEFINE_SPINLOCK(hvcs_structs_lock);

static void hvcs_unthrottle(struct tty_struct *tty);
static void hvcs_throttle(struct tty_struct *tty);
static irqreturn_t hvcs_handle_interrupt(int irq, void *dev_instance);

static int hvcs_write(struct tty_struct *tty,
		const unsigned char *buf, int count);
static int hvcs_write_room(struct tty_struct *tty);
static int hvcs_chars_in_buffer(struct tty_struct *tty);

static int hvcs_has_pi(struct hvcs_struct *hvcsd);
static void hvcs_set_pi(struct hvcs_partner_info *pi,
		struct hvcs_struct *hvcsd);
static int hvcs_get_pi(struct hvcs_struct *hvcsd);
static int hvcs_rescan_devices_list(void);

static int hvcs_partner_connect(struct hvcs_struct *hvcsd);
static void hvcs_partner_free(struct hvcs_struct *hvcsd);

static int hvcs_enable_device(struct hvcs_struct *hvcsd,
		uint32_t unit_address, unsigned int irq, struct vio_dev *dev);

static int hvcs_open(struct tty_struct *tty, struct file *filp);
static void hvcs_close(struct tty_struct *tty, struct file *filp);
static void hvcs_hangup(struct tty_struct * tty);

static int __devinit hvcs_probe(struct vio_dev *dev,
		const struct vio_device_id *id);
static int __devexit hvcs_remove(struct vio_dev *dev);
static int __init hvcs_module_init(void);
static void __exit hvcs_module_exit(void);

#define HVCS_SCHED_READ	0x00000001
#define HVCS_QUICK_READ	0x00000002
#define HVCS_TRY_WRITE	0x00000004
#define HVCS_READ_MASK	(HVCS_SCHED_READ | HVCS_QUICK_READ)

static inline struct hvcs_struct *from_vio_dev(struct vio_dev *viod)
{
	return dev_get_drvdata(&viod->dev);
}
/* The sysfs interface for the driver and devices */

static ssize_t hvcs_partner_vtys_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vio_dev *viod = to_vio_dev(dev);
	struct hvcs_struct *hvcsd = from_vio_dev(viod);
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&hvcsd->lock, flags);
	retval = sprintf(buf, "%X\n", hvcsd->p_unit_address);
	spin_unlock_irqrestore(&hvcsd->lock, flags);
	return retval;
}
static DEVICE_ATTR(partner_vtys, S_IRUGO, hvcs_partner_vtys_show, NULL);

static ssize_t hvcs_partner_clcs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vio_dev *viod = to_vio_dev(dev);
	struct hvcs_struct *hvcsd = from_vio_dev(viod);
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&hvcsd->lock, flags);
	retval = sprintf(buf, "%s\n", &hvcsd->p_location_code[0]);
	spin_unlock_irqrestore(&hvcsd->lock, flags);
	return retval;
}
static DEVICE_ATTR(partner_clcs, S_IRUGO, hvcs_partner_clcs_show, NULL);

static ssize_t hvcs_current_vty_store(struct device *dev, struct device_attribute *attr, const char * buf,
		size_t count)
{
	/*
	 * Don't need this feature at the present time because firmware doesn't
	 * yet support multiple partners.
	 */
	printk(KERN_INFO "HVCS: Denied current_vty change: -EPERM.\n");
	return -EPERM;
}

static ssize_t hvcs_current_vty_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vio_dev *viod = to_vio_dev(dev);
	struct hvcs_struct *hvcsd = from_vio_dev(viod);
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&hvcsd->lock, flags);
	retval = sprintf(buf, "%s\n", &hvcsd->p_location_code[0]);
	spin_unlock_irqrestore(&hvcsd->lock, flags);
	return retval;
}

static DEVICE_ATTR(current_vty,
	S_IRUGO | S_IWUSR, hvcs_current_vty_show, hvcs_current_vty_store);

static ssize_t hvcs_vterm_state_store(struct device *dev, struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct vio_dev *viod = to_vio_dev(dev);
	struct hvcs_struct *hvcsd = from_vio_dev(viod);
	unsigned long flags;

	/* writing a '0' to this sysfs entry will result in the disconnect. */
	if (simple_strtol(buf, NULL, 0) != 0)
		return -EINVAL;

	spin_lock_irqsave(&hvcsd->lock, flags);

	if (hvcsd->open_count > 0) {
		spin_unlock_irqrestore(&hvcsd->lock, flags);
		printk(KERN_INFO "HVCS: vterm state unchanged.  "
				"The hvcs device node is still in use.\n");
		return -EPERM;
	}

	if (hvcsd->connected == 0) {
		spin_unlock_irqrestore(&hvcsd->lock, flags);
		printk(KERN_INFO "HVCS: vterm state unchanged. The"
				" vty-server is not connected to a vty.\n");
		return -EPERM;
	}

	hvcs_partner_free(hvcsd);
	printk(KERN_INFO "HVCS: Closed vty-server@%X and"
			" partner vty@%X:%d connection.\n",
			hvcsd->vdev->unit_address,
			hvcsd->p_unit_address,
			(uint32_t)hvcsd->p_partition_ID);

	spin_unlock_irqrestore(&hvcsd->lock, flags);
	return count;
}

static ssize_t hvcs_vterm_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vio_dev *viod = to_vio_dev(dev);
	struct hvcs_struct *hvcsd = from_vio_dev(viod);
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&hvcsd->lock, flags);
	retval = sprintf(buf, "%d\n", hvcsd->connected);
	spin_unlock_irqrestore(&hvcsd->lock, flags);
	return retval;
}
static DEVICE_ATTR(vterm_state, S_IRUGO | S_IWUSR,
		hvcs_vterm_state_show, hvcs_vterm_state_store);

static ssize_t hvcs_index_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vio_dev *viod = to_vio_dev(dev);
	struct hvcs_struct *hvcsd = from_vio_dev(viod);
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&hvcsd->lock, flags);
	retval = sprintf(buf, "%d\n", hvcsd->index);
	spin_unlock_irqrestore(&hvcsd->lock, flags);
	return retval;
}

static DEVICE_ATTR(index, S_IRUGO, hvcs_index_show, NULL);

static struct attribute *hvcs_attrs[] = {
	&dev_attr_partner_vtys.attr,
	&dev_attr_partner_clcs.attr,
	&dev_attr_current_vty.attr,
	&dev_attr_vterm_state.attr,
	&dev_attr_index.attr,
	NULL,
};

static struct attribute_group hvcs_attr_group = {
	.attrs = hvcs_attrs,
};

static ssize_t hvcs_rescan_show(struct device_driver *ddp, char *buf)
{
	/* A 1 means it is updating, a 0 means it is done updating */
	return snprintf(buf, PAGE_SIZE, "%d\n", hvcs_rescan_status);
}

static ssize_t hvcs_rescan_store(struct device_driver *ddp, const char * buf,
		size_t count)
{
	if ((simple_strtol(buf, NULL, 0) != 1)
		&& (hvcs_rescan_status != 0))
		return -EINVAL;

	hvcs_rescan_status = 1;
	printk(KERN_INFO "HVCS: rescanning partner info for all"
		" vty-servers.\n");
	hvcs_rescan_devices_list();
	hvcs_rescan_status = 0;
	return count;
}

static DRIVER_ATTR(rescan,
	S_IRUGO | S_IWUSR, hvcs_rescan_show, hvcs_rescan_store);

static void hvcs_kick(void)
{
	hvcs_kicked = 1;
	wmb();
	wake_up_process(hvcs_task);
}

static void hvcs_unthrottle(struct tty_struct *tty)
{
	struct hvcs_struct *hvcsd = tty->driver_data;
	unsigned long flags;

	spin_lock_irqsave(&hvcsd->lock, flags);
	hvcsd->todo_mask |= HVCS_SCHED_READ;
	spin_unlock_irqrestore(&hvcsd->lock, flags);
	hvcs_kick();
}

static void hvcs_throttle(struct tty_struct *tty)
{
	struct hvcs_struct *hvcsd = tty->driver_data;
	unsigned long flags;

	spin_lock_irqsave(&hvcsd->lock, flags);
	vio_disable_interrupts(hvcsd->vdev);
	spin_unlock_irqrestore(&hvcsd->lock, flags);
}

static irqreturn_t hvcs_handle_interrupt(int irq, void *dev_instance)
{
	struct hvcs_struct *hvcsd = dev_instance;

	spin_lock(&hvcsd->lock);
	vio_disable_interrupts(hvcsd->vdev);
	hvcsd->todo_mask |= HVCS_SCHED_READ;
	spin_unlock(&hvcsd->lock);
	hvcs_kick();

	return IRQ_HANDLED;
}

/* This function must be called with the hvcsd->lock held */
static void hvcs_try_write(struct hvcs_struct *hvcsd)
{
	uint32_t unit_address = hvcsd->vdev->unit_address;
	struct tty_struct *tty = hvcsd->tty;
	int sent;

	if (hvcsd->todo_mask & HVCS_TRY_WRITE) {
		/* won't send partial writes */
		sent = hvc_put_chars(unit_address,
				&hvcsd->buffer[0],
				hvcsd->chars_in_buffer );
		if (sent > 0) {
			hvcsd->chars_in_buffer = 0;
			/* wmb(); */
			hvcsd->todo_mask &= ~(HVCS_TRY_WRITE);
			/* wmb(); */

			/*
			 * We are still obligated to deliver the data to the
			 * hypervisor even if the tty has been closed because
			 * we commited to delivering it.  But don't try to wake
			 * a non-existent tty.
			 */
			if (tty) {
				tty_wakeup(tty);
			}
		}
	}
}

static int hvcs_io(struct hvcs_struct *hvcsd)
{
	uint32_t unit_address;
	struct tty_struct *tty;
	char buf[HVCS_BUFF_LEN] __ALIGNED__;
	unsigned long flags;
	int got = 0;

	spin_lock_irqsave(&hvcsd->lock, flags);

	unit_address = hvcsd->vdev->unit_address;
	tty = hvcsd->tty;

	hvcs_try_write(hvcsd);

	if (!tty || test_bit(TTY_THROTTLED, &tty->flags)) {
		hvcsd->todo_mask &= ~(HVCS_READ_MASK);
		goto bail;
	} else if (!(hvcsd->todo_mask & (HVCS_READ_MASK)))
		goto bail;

	/* remove the read masks */
	hvcsd->todo_mask &= ~(HVCS_READ_MASK);

	if (tty_buffer_request_room(tty, HVCS_BUFF_LEN) >= HVCS_BUFF_LEN) {
		got = hvc_get_chars(unit_address,
				&buf[0],
				HVCS_BUFF_LEN);
		tty_insert_flip_string(tty, buf, got);
	}

	/* Give the TTY time to process the data we just sent. */
	if (got)
		hvcsd->todo_mask |= HVCS_QUICK_READ;

	spin_unlock_irqrestore(&hvcsd->lock, flags);
	/* This is synch because tty->low_latency == 1 */
	if(got)
		tty_flip_buffer_push(tty);

	if (!got) {
		/* Do this _after_ the flip_buffer_push */
		spin_lock_irqsave(&hvcsd->lock, flags);
		vio_enable_interrupts(hvcsd->vdev);
		spin_unlock_irqrestore(&hvcsd->lock, flags);
	}

	return hvcsd->todo_mask;

 bail:
	spin_unlock_irqrestore(&hvcsd->lock, flags);
	return hvcsd->todo_mask;
}

static int khvcsd(void *unused)
{
	struct hvcs_struct *hvcsd;
	int hvcs_todo_mask;

	__set_current_state(TASK_RUNNING);

	do {
		hvcs_todo_mask = 0;
		hvcs_kicked = 0;
		wmb();

		spin_lock(&hvcs_structs_lock);
		list_for_each_entry(hvcsd, &hvcs_structs, next) {
			hvcs_todo_mask |= hvcs_io(hvcsd);
		}
		spin_unlock(&hvcs_structs_lock);

		/*
		 * If any of the hvcs adapters want to try a write or quick read
		 * don't schedule(), yield a smidgen then execute the hvcs_io
		 * thread again for those that want the write.
		 */
		 if (hvcs_todo_mask & (HVCS_TRY_WRITE | HVCS_QUICK_READ)) {
			yield();
			continue;
		}

		set_current_state(TASK_INTERRUPTIBLE);
		if (!hvcs_kicked)
			schedule();
		__set_current_state(TASK_RUNNING);
	} while (!kthread_should_stop());

	return 0;
}

static struct vio_device_id hvcs_driver_table[] __devinitdata= {
	{"serial-server", "hvterm2"},
	{ "", "" }
};
MODULE_DEVICE_TABLE(vio, hvcs_driver_table);

static void hvcs_return_index(int index)
{
	/* Paranoia check */
	if (!hvcs_index_list)
		return;
	if (index < 0 || index >= hvcs_index_count)
		return;
	if (hvcs_index_list[index] == -1)
		return;
	else
		hvcs_index_list[index] = -1;
}

/* callback when the kref ref count reaches zero */
static void destroy_hvcs_struct(struct kref *kref)
{
	struct hvcs_struct *hvcsd = from_kref(kref);
	struct vio_dev *vdev;
	unsigned long flags;

	spin_lock(&hvcs_structs_lock);
	spin_lock_irqsave(&hvcsd->lock, flags);

	/* the list_del poisons the pointers */
	list_del(&(hvcsd->next));

	if (hvcsd->connected == 1) {
		hvcs_partner_free(hvcsd);
		printk(KERN_INFO "HVCS: Closed vty-server@%X and"
				" partner vty@%X:%d connection.\n",
				hvcsd->vdev->unit_address,
				hvcsd->p_unit_address,
				(uint32_t)hvcsd->p_partition_ID);
	}
	printk(KERN_INFO "HVCS: Destroyed hvcs_struct for vty-server@%X.\n",
			hvcsd->vdev->unit_address);

	vdev = hvcsd->vdev;
	hvcsd->vdev = NULL;

	hvcsd->p_unit_address = 0;
	hvcsd->p_partition_ID = 0;
	hvcs_return_index(hvcsd->index);
	memset(&hvcsd->p_location_code[0], 0x00, HVCS_CLC_LENGTH + 1);

	spin_unlock_irqrestore(&hvcsd->lock, flags);
	spin_unlock(&hvcs_structs_lock);

	sysfs_remove_group(&vdev->dev.kobj, &hvcs_attr_group);

	kfree(hvcsd);
}

static int hvcs_get_index(void)
{
	int i;
	/* Paranoia check */
	if (!hvcs_index_list) {
		printk(KERN_ERR "HVCS: hvcs_index_list NOT valid!.\n");
		return -EFAULT;
	}
	/* Find the numerically lowest first free index. */
	for(i = 0; i < hvcs_index_count; i++) {
		if (hvcs_index_list[i] == -1) {
			hvcs_index_list[i] = 0;
			return i;
		}
	}
	return -1;
}

static int __devinit hvcs_probe(
	struct vio_dev *dev,
	const struct vio_device_id *id)
{
	struct hvcs_struct *hvcsd;
	int index;
	int retval;

	if (!dev || !id) {
		printk(KERN_ERR "HVCS: probed with invalid parameter.\n");
		return -EPERM;
	}

	/* early to avoid cleanup on failure */
	index = hvcs_get_index();
	if (index < 0) {
		return -EFAULT;
	}

	hvcsd = kzalloc(sizeof(*hvcsd), GFP_KERNEL);
	if (!hvcsd)
		return -ENODEV;


	spin_lock_init(&hvcsd->lock);
	/* Automatically incs the refcount the first time */
	kref_init(&hvcsd->kref);

	hvcsd->vdev = dev;
	dev_set_drvdata(&dev->dev, hvcsd);

	hvcsd->index = index;

	/* hvcsd->index = ++hvcs_struct_count; */
	hvcsd->chars_in_buffer = 0;
	hvcsd->todo_mask = 0;
	hvcsd->connected = 0;

	/*
	 * This will populate the hvcs_struct's partner info fields for the
	 * first time.
	 */
	if (hvcs_get_pi(hvcsd)) {
		printk(KERN_ERR "HVCS: Failed to fetch partner"
			" info for vty-server@%X on device probe.\n",
			hvcsd->vdev->unit_address);
	}

	/*
	 * If a user app opens a tty that corresponds to this vty-server before
	 * the hvcs_struct has been added to the devices list then the user app
	 * will get -ENODEV.
	 */
	spin_lock(&hvcs_structs_lock);
	list_add_tail(&(hvcsd->next), &hvcs_structs);
	spin_unlock(&hvcs_structs_lock);

	retval = sysfs_create_group(&dev->dev.kobj, &hvcs_attr_group);
	if (retval) {
		printk(KERN_ERR "HVCS: Can't create sysfs attrs for vty-server@%X\n",
		       hvcsd->vdev->unit_address);
		return retval;
	}

	printk(KERN_INFO "HVCS: vty-server@%X added to the vio bus.\n", dev->unit_address);

	/*
	 * DON'T enable interrupts here because there is no user to receive the
	 * data.
	 */
	return 0;
}

static int __devexit hvcs_remove(struct vio_dev *dev)
{
	struct hvcs_struct *hvcsd = dev_get_drvdata(&dev->dev);
	unsigned long flags;
	struct tty_struct *tty;

	if (!hvcsd)
		return -ENODEV;

	/* By this time the vty-server won't be getting any more interrupts */

	spin_lock_irqsave(&hvcsd->lock, flags);

	tty = hvcsd->tty;

	spin_unlock_irqrestore(&hvcsd->lock, flags);

	/*
	 * Let the last holder of this object cause it to be removed, which
	 * would probably be tty_hangup below.
	 */
	kref_put(&hvcsd->kref, destroy_hvcs_struct);

	/*
	 * The hangup is a scheduled function which will auto chain call
	 * hvcs_hangup.  The tty should always be valid at this time unless a
	 * simultaneous tty close already cleaned up the hvcs_struct.
	 */
	if (tty)
		tty_hangup(tty);

	printk(KERN_INFO "HVCS: vty-server@%X removed from the"
			" vio bus.\n", dev->unit_address);
	return 0;
};

static struct vio_driver hvcs_vio_driver = {
	.id_table	= hvcs_driver_table,
	.probe		= hvcs_probe,
	.remove		= __devexit_p(hvcs_remove),
	.driver		= {
		.name	= hvcs_driver_name,
		.owner	= THIS_MODULE,
	}
};

/* Only called from hvcs_get_pi please */
static void hvcs_set_pi(struct hvcs_partner_info *pi, struct hvcs_struct *hvcsd)
{
	int clclength;

	hvcsd->p_unit_address = pi->unit_address;
	hvcsd->p_partition_ID  = pi->partition_ID;
	clclength = strlen(&pi->location_code[0]);
	if (clclength > HVCS_CLC_LENGTH)
		clclength = HVCS_CLC_LENGTH;

	/* copy the null-term char too */
	strncpy(&hvcsd->p_location_code[0],
			&pi->location_code[0], clclength + 1);
}

static int hvcs_get_pi(struct hvcs_struct *hvcsd)
{
	struct hvcs_partner_info *pi;
	uint32_t unit_address = hvcsd->vdev->unit_address;
	struct list_head head;
	int retval;

	spin_lock(&hvcs_pi_lock);
	if (!hvcs_pi_buff) {
		spin_unlock(&hvcs_pi_lock);
		return -EFAULT;
	}
	retval = hvcs_get_partner_info(unit_address, &head, hvcs_pi_buff);
	spin_unlock(&hvcs_pi_lock);
	if (retval) {
		printk(KERN_ERR "HVCS: Failed to fetch partner"
			" info for vty-server@%x.\n", unit_address);
		return retval;
	}

	/* nixes the values if the partner vty went away */
	hvcsd->p_unit_address = 0;
	hvcsd->p_partition_ID = 0;

	list_for_each_entry(pi, &head, node)
		hvcs_set_pi(pi, hvcsd);

	hvcs_free_partner_info(&head);
	return 0;
}

static int hvcs_rescan_devices_list(void)
{
	struct hvcs_struct *hvcsd;
	unsigned long flags;

	spin_lock(&hvcs_structs_lock);

	list_for_each_entry(hvcsd, &hvcs_structs, next) {
		spin_lock_irqsave(&hvcsd->lock, flags);
		hvcs_get_pi(hvcsd);
		spin_unlock_irqrestore(&hvcsd->lock, flags);
	}

	spin_unlock(&hvcs_structs_lock);

	return 0;
}

static int hvcs_has_pi(struct hvcs_struct *hvcsd)
{
	if ((!hvcsd->p_unit_address) || (!hvcsd->p_partition_ID))
		return 0;
	return 1;
}

static int hvcs_partner_connect(struct hvcs_struct *hvcsd)
{
	int retval;
	unsigned int unit_address = hvcsd->vdev->unit_address;

	/*
	 * If there wasn't any pi when the device was added it doesn't meant
	 * there isn't any now.  This driver isn't notified when a new partner
	 * vty is added to a vty-server so we discover changes on our own.
	 * Please see comments in hvcs_register_connection() for justification
	 * of this bizarre code.
	 */
	retval = hvcs_register_connection(unit_address,
			hvcsd->p_partition_ID,
			hvcsd->p_unit_address);
	if (!retval) {
		hvcsd->connected = 1;
		return 0;
	} else if (retval != -EINVAL)
		return retval;

	/*
	 * As per the spec re-get the pi and try again if -EINVAL after the
	 * first connection attempt.
	 */
	if (hvcs_get_pi(hvcsd))
		return -ENOMEM;

	if (!hvcs_has_pi(hvcsd))
		return -ENODEV;

	retval = hvcs_register_connection(unit_address,
			hvcsd->p_partition_ID,
			hvcsd->p_unit_address);
	if (retval != -EINVAL) {
		hvcsd->connected = 1;
		return retval;
	}

	/*
	 * EBUSY is the most likely scenario though the vty could have been
	 * removed or there really could be an hcall error due to the parameter
	 * data but thanks to ambiguous firmware return codes we can't really
	 * tell.
	 */
	printk(KERN_INFO "HVCS: vty-server or partner"
			" vty is busy.  Try again later.\n");
	return -EBUSY;
}

/* This function must be called with the hvcsd->lock held */
static void hvcs_partner_free(struct hvcs_struct *hvcsd)
{
	int retval;
	do {
		retval = hvcs_free_connection(hvcsd->vdev->unit_address);
	} while (retval == -EBUSY);
	hvcsd->connected = 0;
}

/* This helper function must be called WITHOUT the hvcsd->lock held */
static int hvcs_enable_device(struct hvcs_struct *hvcsd, uint32_t unit_address,
		unsigned int irq, struct vio_dev *vdev)
{
	unsigned long flags;
	int rc;

	/*
	 * It is possible that the vty-server was removed between the time that
	 * the conn was registered and now.
	 */
	if (!(rc = request_irq(irq, &hvcs_handle_interrupt,
				IRQF_DISABLED, "ibmhvcs", hvcsd))) {
		/*
		 * It is possible the vty-server was removed after the irq was
		 * requested but before we have time to enable interrupts.
		 */
		if (vio_enable_interrupts(vdev) == H_SUCCESS)
			return 0;
		else {
			printk(KERN_ERR "HVCS: int enable failed for"
					" vty-server@%X.\n", unit_address);
			free_irq(irq, hvcsd);
		}
	} else
		printk(KERN_ERR "HVCS: irq req failed for"
				" vty-server@%X.\n", unit_address);

	spin_lock_irqsave(&hvcsd->lock, flags);
	hvcs_partner_free(hvcsd);
	spin_unlock_irqrestore(&hvcsd->lock, flags);

	return rc;

}

static struct hvcs_struct *hvcs_get_by_index(int index)
{
	struct hvcs_struct *hvcsd = NULL;
	unsigned long flags;

	spin_lock(&hvcs_structs_lock);
	/* We can immediately discard OOB requests */
	if (index >= 0 && index < HVCS_MAX_SERVER_ADAPTERS) {
		list_for_each_entry(hvcsd, &hvcs_structs, next) {
			spin_lock_irqsave(&hvcsd->lock, flags);
			if (hvcsd->index == index) {
				kref_get(&hvcsd->kref);
				spin_unlock_irqrestore(&hvcsd->lock, flags);
				spin_unlock(&hvcs_structs_lock);
				return hvcsd;
			}
			spin_unlock_irqrestore(&hvcsd->lock, flags);
		}
		hvcsd = NULL;
	}

	spin_unlock(&hvcs_structs_lock);
	return hvcsd;
}

static int hvcs_open(struct tty_struct *tty, struct file *filp)
{
	struct hvcs_struct *hvcsd;
	int rc, retval = 0;
	unsigned long flags;
	unsigned int irq;
	struct vio_dev *vdev;
	unsigned long unit_address;

	if (tty->driver_data)
		goto fast_open;

	/*
	 * Is there a vty-server that shares the same index?
	 * This function increments the kref index.
	 */
	if (!(hvcsd = hvcs_get_by_index(tty->index))) {
		printk(KERN_WARNING "HVCS: open failed, no device associated"
				" with tty->index %d.\n", tty->index);
		return -ENODEV;
	}

	spin_lock_irqsave(&hvcsd->lock, flags);

	if (hvcsd->connected == 0)
		if ((retval = hvcs_partner_connect(hvcsd)))
			goto error_release;

	hvcsd->open_count = 1;
	hvcsd->tty = tty;
	tty->driver_data = hvcsd;

	memset(&hvcsd->buffer[0], 0x00, HVCS_BUFF_LEN);

	/*
	 * Save these in the spinlock for the enable operations that need them
	 * outside of the spinlock.
	 */
	irq = hvcsd->vdev->irq;
	vdev = hvcsd->vdev;
	unit_address = hvcsd->vdev->unit_address;

	hvcsd->todo_mask |= HVCS_SCHED_READ;
	spin_unlock_irqrestore(&hvcsd->lock, flags);

	/*
	 * This must be done outside of the spinlock because it requests irqs
	 * and will grab the spinlock and free the connection if it fails.
	 */
	if (((rc = hvcs_enable_device(hvcsd, unit_address, irq, vdev)))) {
		kref_put(&hvcsd->kref, destroy_hvcs_struct);
		printk(KERN_WARNING "HVCS: enable device failed.\n");
		return rc;
	}

	goto open_success;

fast_open:
	hvcsd = tty->driver_data;

	spin_lock_irqsave(&hvcsd->lock, flags);
	kref_get(&hvcsd->kref);
	hvcsd->open_count++;
	hvcsd->todo_mask |= HVCS_SCHED_READ;
	spin_unlock_irqrestore(&hvcsd->lock, flags);

open_success:
	hvcs_kick();

	printk(KERN_INFO "HVCS: vty-server@%X connection opened.\n",
		hvcsd->vdev->unit_address );

	return 0;

error_release:
	spin_unlock_irqrestore(&hvcsd->lock, flags);
	kref_put(&hvcsd->kref, destroy_hvcs_struct);

	printk(KERN_WARNING "HVCS: partner connect failed.\n");
	return retval;
}

static void hvcs_close(struct tty_struct *tty, struct file *filp)
{
	struct hvcs_struct *hvcsd;
	unsigned long flags;
	int irq = NO_IRQ;

	/*
	 * Is someone trying to close the file associated with this device after
	 * we have hung up?  If so tty->driver_data wouldn't be valid.
	 */
	if (tty_hung_up_p(filp))
		return;

	/*
	 * No driver_data means that this close was probably issued after a
	 * failed hvcs_open by the tty layer's release_dev() api and we can just
	 * exit cleanly.
	 */
	if (!tty->driver_data)
		return;

	hvcsd = tty->driver_data;

	spin_lock_irqsave(&hvcsd->lock, flags);
	if (--hvcsd->open_count == 0) {

		vio_disable_interrupts(hvcsd->vdev);

		/*
		 * NULL this early so that the kernel_thread doesn't try to
		 * execute any operations on the TTY even though it is obligated
		 * to deliver any pending I/O to the hypervisor.
		 */
		hvcsd->tty = NULL;

		irq = hvcsd->vdev->irq;
		spin_unlock_irqrestore(&hvcsd->lock, flags);

		tty_wait_until_sent(tty, HVCS_CLOSE_WAIT);

		/*
		 * This line is important because it tells hvcs_open that this
		 * device needs to be re-configured the next time hvcs_open is
		 * called.
		 */
		tty->driver_data = NULL;

		free_irq(irq, hvcsd);
		kref_put(&hvcsd->kref, destroy_hvcs_struct);
		return;
	} else if (hvcsd->open_count < 0) {
		printk(KERN_ERR "HVCS: vty-server@%X open_count: %d"
				" is missmanaged.\n",
		hvcsd->vdev->unit_address, hvcsd->open_count);
	}

	spin_unlock_irqrestore(&hvcsd->lock, flags);
	kref_put(&hvcsd->kref, destroy_hvcs_struct);
}

static void hvcs_hangup(struct tty_struct * tty)
{
	struct hvcs_struct *hvcsd = tty->driver_data;
	unsigned long flags;
	int temp_open_count;
	int irq = NO_IRQ;

	spin_lock_irqsave(&hvcsd->lock, flags);
	/* Preserve this so that we know how many kref refs to put */
	temp_open_count = hvcsd->open_count;

	/*
	 * Don't kref put inside the spinlock because the destruction
	 * callback may use the spinlock and it may get called before the
	 * spinlock has been released.
	 */
	vio_disable_interrupts(hvcsd->vdev);

	hvcsd->todo_mask = 0;

	/* I don't think the tty needs the hvcs_struct pointer after a hangup */
	hvcsd->tty->driver_data = NULL;
	hvcsd->tty = NULL;

	hvcsd->open_count = 0;

	/* This will drop any buffered data on the floor which is OK in a hangup
	 * scenario. */
	memset(&hvcsd->buffer[0], 0x00, HVCS_BUFF_LEN);
	hvcsd->chars_in_buffer = 0;

	irq = hvcsd->vdev->irq;

	spin_unlock_irqrestore(&hvcsd->lock, flags);

	free_irq(irq, hvcsd);

	/*
	 * We need to kref_put() for every open_count we have since the
	 * tty_hangup() function doesn't invoke a close per open connection on a
	 * non-console device.
	 */
	while(temp_open_count) {
		--temp_open_count;
		/*
		 * The final put will trigger destruction of the hvcs_struct.
		 * NOTE:  If this hangup was signaled from user space then the
		 * final put will never happen.
		 */
		kref_put(&hvcsd->kref, destroy_hvcs_struct);
	}
}

static int hvcs_write(struct tty_struct *tty,
		const unsigned char *buf, int count)
{
	struct hvcs_struct *hvcsd = tty->driver_data;
	unsigned int unit_address;
	const unsigned char *charbuf;
	unsigned long flags;
	int total_sent = 0;
	int tosend = 0;
	int result = 0;

	/*
	 * If they don't check the return code off of their open they may
	 * attempt this even if there is no connected device.
	 */
	if (!hvcsd)
		return -ENODEV;

	/* Reasonable size to prevent user level flooding */
	if (count > HVCS_MAX_FROM_USER) {
		printk(KERN_WARNING "HVCS write: count being truncated to"
				" HVCS_MAX_FROM_USER.\n");
		count = HVCS_MAX_FROM_USER;
	}

	charbuf = buf;

	spin_lock_irqsave(&hvcsd->lock, flags);

	/*
	 * Somehow an open succedded but the device was removed or the
	 * connection terminated between the vty-server and partner vty during
	 * the middle of a write operation?  This is a crummy place to do this
	 * but we want to keep it all in the spinlock.
	 */
	if (hvcsd->open_count <= 0) {
		spin_unlock_irqrestore(&hvcsd->lock, flags);
		return -ENODEV;
	}

	unit_address = hvcsd->vdev->unit_address;

	while (count > 0) {
		tosend = min(count, (HVCS_BUFF_LEN - hvcsd->chars_in_buffer));
		/*
		 * No more space, this probably means that the last call to
		 * hvcs_write() didn't succeed and the buffer was filled up.
		 */
		if (!tosend)
			break;

		memcpy(&hvcsd->buffer[hvcsd->chars_in_buffer],
				&charbuf[total_sent],
				tosend);

		hvcsd->chars_in_buffer += tosend;

		result = 0;

		/*
		 * If this is true then we don't want to try writing to the
		 * hypervisor because that is the kernel_threads job now.  We'll
		 * just add to the buffer.
		 */
		if (!(hvcsd->todo_mask & HVCS_TRY_WRITE))
			/* won't send partial writes */
			result = hvc_put_chars(unit_address,
					&hvcsd->buffer[0],
					hvcsd->chars_in_buffer);

		/*
		 * Since we know we have enough room in hvcsd->buffer for
		 * tosend we record that it was sent regardless of whether the
		 * hypervisor actually took it because we have it buffered.
		 */
		total_sent+=tosend;
		count-=tosend;
		if (result == 0) {
			hvcsd->todo_mask |= HVCS_TRY_WRITE;
			hvcs_kick();
			break;
		}

		hvcsd->chars_in_buffer = 0;
		/*
		 * Test after the chars_in_buffer reset otherwise this could
		 * deadlock our writes if hvc_put_chars fails.
		 */
		if (result < 0)
			break;
	}

	spin_unlock_irqrestore(&hvcsd->lock, flags);

	if (result == -1)
		return -EIO;
	else
		return total_sent;
}

static int hvcs_write_room(struct tty_struct *tty)
{
	struct hvcs_struct *hvcsd = tty->driver_data;

	if (!hvcsd || hvcsd->open_count <= 0)
		return 0;

	return HVCS_BUFF_LEN - hvcsd->chars_in_buffer;
}

static int hvcs_chars_in_buffer(struct tty_struct *tty)
{
	struct hvcs_struct *hvcsd = tty->driver_data;

	return hvcsd->chars_in_buffer;
}

static const struct tty_operations hvcs_ops = {
	.open = hvcs_open,
	.close = hvcs_close,
	.hangup = hvcs_hangup,
	.write = hvcs_write,
	.write_room = hvcs_write_room,
	.chars_in_buffer = hvcs_chars_in_buffer,
	.unthrottle = hvcs_unthrottle,
	.throttle = hvcs_throttle,
};

static int hvcs_alloc_index_list(int n)
{
	int i;

	hvcs_index_list = kmalloc(n * sizeof(hvcs_index_count),GFP_KERNEL);
	if (!hvcs_index_list)
		return -ENOMEM;
	hvcs_index_count = n;
	for (i = 0; i < hvcs_index_count; i++)
		hvcs_index_list[i] = -1;
	return 0;
}

static void hvcs_free_index_list(void)
{
	/* Paranoia check to be thorough. */
	kfree(hvcs_index_list);
	hvcs_index_list = NULL;
	hvcs_index_count = 0;
}

static int __init hvcs_module_init(void)
{
	int rc;
	int num_ttys_to_alloc;

	printk(KERN_INFO "Initializing %s\n", hvcs_driver_string);

	/* Has the user specified an overload with an insmod param? */
	if (hvcs_parm_num_devs <= 0 ||
		(hvcs_parm_num_devs > HVCS_MAX_SERVER_ADAPTERS)) {
		num_ttys_to_alloc = HVCS_DEFAULT_SERVER_ADAPTERS;
	} else
		num_ttys_to_alloc = hvcs_parm_num_devs;

	hvcs_tty_driver = alloc_tty_driver(num_ttys_to_alloc);
	if (!hvcs_tty_driver)
		return -ENOMEM;

	if (hvcs_alloc_index_list(num_ttys_to_alloc)) {
		rc = -ENOMEM;
		goto index_fail;
	}

	hvcs_tty_driver->owner = THIS_MODULE;

	hvcs_tty_driver->driver_name = hvcs_driver_name;
	hvcs_tty_driver->name = hvcs_device_node;

	/*
	 * We'll let the system assign us a major number, indicated by leaving
	 * it blank.
	 */

	hvcs_tty_driver->minor_start = HVCS_MINOR_START;
	hvcs_tty_driver->type = TTY_DRIVER_TYPE_SYSTEM;

	/*
	 * We role our own so that we DONT ECHO.  We can't echo because the
	 * device we are connecting to already echoes by default and this would
	 * throw us into a horrible recursive echo-echo-echo loop.
	 */
	hvcs_tty_driver->init_termios = hvcs_tty_termios;
	hvcs_tty_driver->flags = TTY_DRIVER_REAL_RAW;

	tty_set_operations(hvcs_tty_driver, &hvcs_ops);

	/*
	 * The following call will result in sysfs entries that denote the
	 * dynamically assigned major and minor numbers for our devices.
	 */
	if (tty_register_driver(hvcs_tty_driver)) {
		printk(KERN_ERR "HVCS: registration as a tty driver failed.\n");
		rc = -EIO;
		goto register_fail;
	}

	hvcs_pi_buff = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!hvcs_pi_buff) {
		rc = -ENOMEM;
		goto buff_alloc_fail;
	}

	hvcs_task = kthread_run(khvcsd, NULL, "khvcsd");
	if (IS_ERR(hvcs_task)) {
		printk(KERN_ERR "HVCS: khvcsd creation failed.  Driver not loaded.\n");
		rc = -EIO;
		goto kthread_fail;
	}

	rc = vio_register_driver(&hvcs_vio_driver);
	if (rc) {
		printk(KERN_ERR "HVCS: can't register vio driver\n");
		goto vio_fail;
	}

	/*
	 * This needs to be done AFTER the vio_register_driver() call or else
	 * the kobjects won't be initialized properly.
	 */
	rc = driver_create_file(&(hvcs_vio_driver.driver), &driver_attr_rescan);
	if (rc) {
		printk(KERN_ERR "HVCS: sysfs attr create failed\n");
		goto attr_fail;
	}

	printk(KERN_INFO "HVCS: driver module inserted.\n");

	return 0;

attr_fail:
	vio_unregister_driver(&hvcs_vio_driver);
vio_fail:
	kthread_stop(hvcs_task);
kthread_fail:
	kfree(hvcs_pi_buff);
buff_alloc_fail:
	tty_unregister_driver(hvcs_tty_driver);
register_fail:
	hvcs_free_index_list();
index_fail:
	put_tty_driver(hvcs_tty_driver);
	hvcs_tty_driver = NULL;
	return rc;
}

static void __exit hvcs_module_exit(void)
{
	/*
	 * This driver receives hvcs_remove callbacks for each device upon
	 * module removal.
	 */

	/*
	 * This synchronous operation  will wake the khvcsd kthread if it is
	 * asleep and will return when khvcsd has terminated.
	 */
	kthread_stop(hvcs_task);

	spin_lock(&hvcs_pi_lock);
	kfree(hvcs_pi_buff);
	hvcs_pi_buff = NULL;
	spin_unlock(&hvcs_pi_lock);

	driver_remove_file(&hvcs_vio_driver.driver, &driver_attr_rescan);

	vio_unregister_driver(&hvcs_vio_driver);

	tty_unregister_driver(hvcs_tty_driver);

	hvcs_free_index_list();

	put_tty_driver(hvcs_tty_driver);

	printk(KERN_INFO "HVCS: driver module removed.\n");
}

module_init(hvcs_module_init);
module_exit(hvcs_module_exit);
