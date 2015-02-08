
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/ioport.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <asm/watchdog.h>

#define PFX "shwdt: "

static int clock_division_ratio = WTCSR_CKS_4096;

#define next_ping_period(cks)	msecs_to_jiffies(cks - 4)

static void sh_wdt_ping(unsigned long data);

static unsigned long shwdt_is_open;
static const struct watchdog_info sh_wdt_info;
static char shwdt_expect_close;
static DEFINE_TIMER(timer, sh_wdt_ping, 0, 0);
static unsigned long next_heartbeat;
static DEFINE_SPINLOCK(shwdt_lock);

#define WATCHDOG_HEARTBEAT 30			/* 30 sec default heartbeat */
static int heartbeat = WATCHDOG_HEARTBEAT;	/* in seconds */

static int nowayout = WATCHDOG_NOWAYOUT;

static void sh_wdt_start(void)
{
	__u8 csr;
	unsigned long flags;

	spin_lock_irqsave(&shwdt_lock, flags);

	next_heartbeat = jiffies + (heartbeat * HZ);
	mod_timer(&timer, next_ping_period(clock_division_ratio));

	csr = sh_wdt_read_csr();
	csr |= WTCSR_WT | clock_division_ratio;
	sh_wdt_write_csr(csr);

	sh_wdt_write_cnt(0);

	/*
	 * These processors have a bit of an inconsistent initialization
	 * process.. starting with SH-3, RSTS was moved to WTCSR, and the
	 * RSTCSR register was removed.
	 *
	 * On the SH-2 however, in addition with bits being in different
	 * locations, we must deal with RSTCSR outright..
	 */
	csr = sh_wdt_read_csr();
	csr |= WTCSR_TME;
	csr &= ~WTCSR_RSTS;
	sh_wdt_write_csr(csr);

#ifdef CONFIG_CPU_SH2
	/*
	 * Whoever came up with the RSTCSR semantics must've been smoking
	 * some of the good stuff, since in addition to the WTCSR/WTCNT write
	 * brain-damage, it's managed to fuck things up one step further..
	 *
	 * If we need to clear the WOVF bit, the upper byte has to be 0xa5..
	 * but if we want to touch RSTE or RSTS, the upper byte has to be
	 * 0x5a..
	 */
	csr = sh_wdt_read_rstcsr();
	csr &= ~RSTCSR_RSTS;
	sh_wdt_write_rstcsr(csr);
#endif
	spin_unlock_irqrestore(&shwdt_lock, flags);
}

static void sh_wdt_stop(void)
{
	__u8 csr;
	unsigned long flags;

	spin_lock_irqsave(&shwdt_lock, flags);

	del_timer(&timer);

	csr = sh_wdt_read_csr();
	csr &= ~WTCSR_TME;
	sh_wdt_write_csr(csr);
	spin_unlock_irqrestore(&shwdt_lock, flags);
}

static inline void sh_wdt_keepalive(void)
{
	unsigned long flags;

	spin_lock_irqsave(&shwdt_lock, flags);
	next_heartbeat = jiffies + (heartbeat * HZ);
	spin_unlock_irqrestore(&shwdt_lock, flags);
}

static int sh_wdt_set_heartbeat(int t)
{
	unsigned long flags;

	if (unlikely(t < 1 || t > 3600)) /* arbitrary upper limit */
		return -EINVAL;

	spin_lock_irqsave(&shwdt_lock, flags);
	heartbeat = t;
	spin_unlock_irqrestore(&shwdt_lock, flags);
	return 0;
}

static void sh_wdt_ping(unsigned long data)
{
	unsigned long flags;

	spin_lock_irqsave(&shwdt_lock, flags);
	if (time_before(jiffies, next_heartbeat)) {
		__u8 csr;

		csr = sh_wdt_read_csr();
		csr &= ~WTCSR_IOVF;
		sh_wdt_write_csr(csr);

		sh_wdt_write_cnt(0);

		mod_timer(&timer, next_ping_period(clock_division_ratio));
	} else
		printk(KERN_WARNING PFX "Heartbeat lost! Will not ping "
		       "the watchdog\n");
	spin_unlock_irqrestore(&shwdt_lock, flags);
}

static int sh_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &shwdt_is_open))
		return -EBUSY;
	if (nowayout)
		__module_get(THIS_MODULE);

	sh_wdt_start();

	return nonseekable_open(inode, file);
}

static int sh_wdt_close(struct inode *inode, struct file *file)
{
	if (shwdt_expect_close == 42) {
		sh_wdt_stop();
	} else {
		printk(KERN_CRIT PFX "Unexpected close, not "
		       "stopping watchdog!\n");
		sh_wdt_keepalive();
	}

	clear_bit(0, &shwdt_is_open);
	shwdt_expect_close = 0;

	return 0;
}

static ssize_t sh_wdt_write(struct file *file, const char *buf,
			    size_t count, loff_t *ppos)
{
	if (count) {
		if (!nowayout) {
			size_t i;

			shwdt_expect_close = 0;

			for (i = 0; i != count; i++) {
				char c;
				if (get_user(c, buf + i))
					return -EFAULT;
				if (c == 'V')
					shwdt_expect_close = 42;
			}
		}
		sh_wdt_keepalive();
	}

	return count;
}

static int sh_wdt_mmap(struct file *file, struct vm_area_struct *vma)
{
	int ret = -ENOSYS;

#ifdef CONFIG_SH_WDT_MMAP
	unsigned long addr;

	/* Only support the simple cases where we map in a register page. */
	if (((vma->vm_end - vma->vm_start) != PAGE_SIZE) || vma->vm_pgoff)
		return -EINVAL;

	/*
	 * Pick WTCNT as the start, it's usually the first register after the
	 * FRQCR, and neither one are generally page-aligned out of the box.
	 */
	addr = WTCNT & ~(PAGE_SIZE - 1);

	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (io_remap_pfn_range(vma, vma->vm_start, addr >> PAGE_SHIFT,
			       PAGE_SIZE, vma->vm_page_prot)) {
		printk(KERN_ERR PFX "%s: io_remap_pfn_range failed\n",
		       __func__);
		return -EAGAIN;
	}

	ret = 0;
#endif

	return ret;
}

static long sh_wdt_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	int new_heartbeat;
	int options, retval = -EINVAL;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user((struct watchdog_info *)arg,
			  &sh_wdt_info, sizeof(sh_wdt_info)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, (int *)arg);
	case WDIOC_SETOPTIONS:
		if (get_user(options, (int *)arg))
			return -EFAULT;

		if (options & WDIOS_DISABLECARD) {
			sh_wdt_stop();
			retval = 0;
		}

		if (options & WDIOS_ENABLECARD) {
			sh_wdt_start();
			retval = 0;
		}

		return retval;
	case WDIOC_KEEPALIVE:
		sh_wdt_keepalive();
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_heartbeat, (int *)arg))
			return -EFAULT;

		if (sh_wdt_set_heartbeat(new_heartbeat))
			return -EINVAL;

		sh_wdt_keepalive();
		/* Fall */
	case WDIOC_GETTIMEOUT:
		return put_user(heartbeat, (int *)arg);
	default:
		return -ENOTTY;
	}
	return 0;
}

static int sh_wdt_notify_sys(struct notifier_block *this,
			     unsigned long code, void *unused)
{
	if (code == SYS_DOWN || code == SYS_HALT)
		sh_wdt_stop();

	return NOTIFY_DONE;
}

static const struct file_operations sh_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= sh_wdt_write,
	.unlocked_ioctl	= sh_wdt_ioctl,
	.open		= sh_wdt_open,
	.release	= sh_wdt_close,
	.mmap		= sh_wdt_mmap,
};

static const struct watchdog_info sh_wdt_info = {
	.options		= WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT |
				  WDIOF_MAGICCLOSE,
	.firmware_version	= 1,
	.identity		= "SH WDT",
};

static struct notifier_block sh_wdt_notifier = {
	.notifier_call		= sh_wdt_notify_sys,
};

static struct miscdevice sh_wdt_miscdev = {
	.minor		= WATCHDOG_MINOR,
	.name		= "watchdog",
	.fops		= &sh_wdt_fops,
};

static int __init sh_wdt_init(void)
{
	int rc;

	if (clock_division_ratio < 0x5 || clock_division_ratio > 0x7) {
		clock_division_ratio = WTCSR_CKS_4096;
		printk(KERN_INFO PFX
		  "clock_division_ratio value must be 0x5<=x<=0x7, using %d\n",
				clock_division_ratio);
	}

	rc = sh_wdt_set_heartbeat(heartbeat);
	if (unlikely(rc)) {
		heartbeat = WATCHDOG_HEARTBEAT;
		printk(KERN_INFO PFX
			"heartbeat value must be 1<=x<=3600, using %d\n",
								heartbeat);
	}

	rc = register_reboot_notifier(&sh_wdt_notifier);
	if (unlikely(rc)) {
		printk(KERN_ERR PFX
			"Can't register reboot notifier (err=%d)\n", rc);
		return rc;
	}

	rc = misc_register(&sh_wdt_miscdev);
	if (unlikely(rc)) {
		printk(KERN_ERR PFX
			"Can't register miscdev on minor=%d (err=%d)\n",
						sh_wdt_miscdev.minor, rc);
		unregister_reboot_notifier(&sh_wdt_notifier);
		return rc;
	}

	printk(KERN_INFO PFX "initialized. heartbeat=%d sec (nowayout=%d)\n",
		heartbeat, nowayout);

	return 0;
}

static void __exit sh_wdt_exit(void)
{
	misc_deregister(&sh_wdt_miscdev);
	unregister_reboot_notifier(&sh_wdt_notifier);
}

MODULE_AUTHOR("Paul Mundt <lethal@linux-sh.org>");
MODULE_DESCRIPTION("SuperH watchdog driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);

module_param(clock_division_ratio, int, 0);
MODULE_PARM_DESC(clock_division_ratio,
	"Clock division ratio. Valid ranges are from 0x5 (1.31ms) "
	"to 0x7 (5.25ms). (default=" __MODULE_STRING(WTCSR_CKS_4096) ")");

module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat,
	"Watchdog heartbeat in seconds. (1 <= heartbeat <= 3600, default="
				__MODULE_STRING(WATCHDOG_HEARTBEAT) ")");

module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout,
	"Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

module_init(sh_wdt_init);
module_exit(sh_wdt_exit);
