

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include <asm/system.h>

#define WDT_IS_PCI
#include "wd501p.h"

#define PFX "wdt_pci: "


#ifndef PCI_VENDOR_ID_ACCESSIO
#define PCI_VENDOR_ID_ACCESSIO 0x494f
#endif
#ifndef PCI_DEVICE_ID_WDG_CSM
#define PCI_DEVICE_ID_WDG_CSM 0x22c0
#endif

/* We can only use 1 card due to the /dev/watchdog restriction */
static int dev_count;

static unsigned long open_lock;
static DEFINE_SPINLOCK(wdtpci_lock);
static char expect_close;

static resource_size_t io;
static int irq;

/* Default timeout */
#define WD_TIMO 60			/* Default heartbeat = 60 seconds */

static int heartbeat = WD_TIMO;
static int wd_heartbeat;
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat,
		"Watchdog heartbeat in seconds. (0<heartbeat<65536, default="
				__MODULE_STRING(WD_TIMO) ")");

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout,
		"Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

/* Support for the Fan Tachometer on the PCI-WDT501 */
static int tachometer;
module_param(tachometer, int, 0);
MODULE_PARM_DESC(tachometer,
		"PCI-WDT501 Fan Tachometer support (0=disable, default=0)");

static int type = 500;
module_param(type, int, 0);
MODULE_PARM_DESC(type,
		"PCI-WDT501 Card type (500 or 501 , default=500)");


static void wdtpci_ctr_mode(int ctr, int mode)
{
	ctr <<= 6;
	ctr |= 0x30;
	ctr |= (mode << 1);
	outb(ctr, WDT_CR);
	udelay(8);
}

static void wdtpci_ctr_load(int ctr, int val)
{
	outb(val & 0xFF, WDT_COUNT0 + ctr);
	udelay(8);
	outb(val >> 8, WDT_COUNT0 + ctr);
	udelay(8);
}


static int wdtpci_start(void)
{
	unsigned long flags;

	spin_lock_irqsave(&wdtpci_lock, flags);

	/*
	 * "pet" the watchdog, as Access says.
	 * This resets the clock outputs.
	 */
	inb(WDT_DC);			/* Disable watchdog */
	udelay(8);
	wdtpci_ctr_mode(2, 0);		/* Program CTR2 for Mode 0:
						Pulse on Terminal Count */
	outb(0, WDT_DC);		/* Enable watchdog */
	udelay(8);
	inb(WDT_DC);			/* Disable watchdog */
	udelay(8);
	outb(0, WDT_CLOCK);		/* 2.0833MHz clock */
	udelay(8);
	inb(WDT_BUZZER);		/* disable */
	udelay(8);
	inb(WDT_OPTONOTRST);		/* disable */
	udelay(8);
	inb(WDT_OPTORST);		/* disable */
	udelay(8);
	inb(WDT_PROGOUT);		/* disable */
	udelay(8);
	wdtpci_ctr_mode(0, 3);		/* Program CTR0 for Mode 3:
						Square Wave Generator */
	wdtpci_ctr_mode(1, 2);		/* Program CTR1 for Mode 2:
						Rate Generator */
	wdtpci_ctr_mode(2, 1);		/* Program CTR2 for Mode 1:
						Retriggerable One-Shot */
	wdtpci_ctr_load(0, 20833);	/* count at 100Hz */
	wdtpci_ctr_load(1, wd_heartbeat);/* Heartbeat */
	/* DO NOT LOAD CTR2 on PCI card! -- JPN */
	outb(0, WDT_DC);		/* Enable watchdog */
	udelay(8);

	spin_unlock_irqrestore(&wdtpci_lock, flags);
	return 0;
}


static int wdtpci_stop(void)
{
	unsigned long flags;

	/* Turn the card off */
	spin_lock_irqsave(&wdtpci_lock, flags);
	inb(WDT_DC);			/* Disable watchdog */
	udelay(8);
	wdtpci_ctr_load(2, 0);		/* 0 length reset pulses now */
	spin_unlock_irqrestore(&wdtpci_lock, flags);
	return 0;
}


static int wdtpci_ping(void)
{
	unsigned long flags;

	spin_lock_irqsave(&wdtpci_lock, flags);
	/* Write a watchdog value */
	inb(WDT_DC);			/* Disable watchdog */
	udelay(8);
	wdtpci_ctr_mode(1, 2);		/* Re-Program CTR1 for Mode 2:
							Rate Generator */
	wdtpci_ctr_load(1, wd_heartbeat);/* Heartbeat */
	outb(0, WDT_DC);		/* Enable watchdog */
	udelay(8);
	spin_unlock_irqrestore(&wdtpci_lock, flags);
	return 0;
}

static int wdtpci_set_heartbeat(int t)
{
	/* Arbitrary, can't find the card's limits */
	if (t < 1 || t > 65535)
		return -EINVAL;

	heartbeat = t;
	wd_heartbeat = t * 100;
	return 0;
}


static int wdtpci_get_status(int *status)
{
	unsigned char new_status;
	unsigned long flags;

	spin_lock_irqsave(&wdtpci_lock, flags);
	new_status = inb(WDT_SR);
	spin_unlock_irqrestore(&wdtpci_lock, flags);

	*status = 0;
	if (new_status & WDC_SR_ISOI0)
		*status |= WDIOF_EXTERN1;
	if (new_status & WDC_SR_ISII1)
		*status |= WDIOF_EXTERN2;
	if (type == 501) {
		if (!(new_status & WDC_SR_TGOOD))
			*status |= WDIOF_OVERHEAT;
		if (!(new_status & WDC_SR_PSUOVER))
			*status |= WDIOF_POWEROVER;
		if (!(new_status & WDC_SR_PSUUNDR))
			*status |= WDIOF_POWERUNDER;
		if (tachometer) {
			if (!(new_status & WDC_SR_FANGOOD))
				*status |= WDIOF_FANFAULT;
		}
	}
	return 0;
}


static int wdtpci_get_temperature(int *temperature)
{
	unsigned short c;
	unsigned long flags;
	spin_lock_irqsave(&wdtpci_lock, flags);
	c = inb(WDT_RT);
	udelay(8);
	spin_unlock_irqrestore(&wdtpci_lock, flags);
	*temperature = (c * 11 / 15) + 7;
	return 0;
}


static irqreturn_t wdtpci_interrupt(int irq, void *dev_id)
{
	/*
	 *	Read the status register see what is up and
	 *	then printk it.
	 */
	unsigned char status;

	spin_lock(&wdtpci_lock);

	status = inb(WDT_SR);
	udelay(8);

	printk(KERN_CRIT PFX "status %d\n", status);

	if (type == 501) {
		if (!(status & WDC_SR_TGOOD)) {
			printk(KERN_CRIT PFX "Overheat alarm.(%d)\n",
								inb(WDT_RT));
			udelay(8);
		}
		if (!(status & WDC_SR_PSUOVER))
			printk(KERN_CRIT PFX "PSU over voltage.\n");
		if (!(status & WDC_SR_PSUUNDR))
			printk(KERN_CRIT PFX "PSU under voltage.\n");
		if (tachometer) {
			if (!(status & WDC_SR_FANGOOD))
				printk(KERN_CRIT PFX "Possible fan fault.\n");
		}
	}
	if (!(status & WDC_SR_WCCR)) {
#ifdef SOFTWARE_REBOOT
#ifdef ONLY_TESTING
		printk(KERN_CRIT PFX "Would Reboot.\n");
#else
		printk(KERN_CRIT PFX "Initiating system reboot.\n");
		emergency_restart(NULL);
#endif
#else
		printk(KERN_CRIT PFX "Reset in 5ms.\n");
#endif
	}
	spin_unlock(&wdtpci_lock);
	return IRQ_HANDLED;
}



static ssize_t wdtpci_write(struct file *file, const char __user *buf,
						size_t count, loff_t *ppos)
{
	if (count) {
		if (!nowayout) {
			size_t i;

			/* In case it was set long ago */
			expect_close = 0;

			for (i = 0; i != count; i++) {
				char c;
				if (get_user(c, buf + i))
					return -EFAULT;
				if (c == 'V')
					expect_close = 42;
			}
		}
		wdtpci_ping();
	}
	return count;
}


static long wdtpci_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_heartbeat;
	int status;

	struct watchdog_info ident = {
		.options =		WDIOF_SETTIMEOUT|
					WDIOF_MAGICCLOSE|
					WDIOF_KEEPALIVEPING,
		.firmware_version =	1,
		.identity =		"PCI-WDT500/501",
	};

	/* Add options according to the card we have */
	ident.options |= (WDIOF_EXTERN1|WDIOF_EXTERN2);
	if (type == 501) {
		ident.options |= (WDIOF_OVERHEAT|WDIOF_POWERUNDER|
							WDIOF_POWEROVER);
		if (tachometer)
			ident.options |= WDIOF_FANFAULT;
	}

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &ident, sizeof(ident)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
		wdtpci_get_status(&status);
		return put_user(status, p);
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_KEEPALIVE:
		wdtpci_ping();
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_heartbeat, p))
			return -EFAULT;
		if (wdtpci_set_heartbeat(new_heartbeat))
			return -EINVAL;
		wdtpci_ping();
		/* Fall */
	case WDIOC_GETTIMEOUT:
		return put_user(heartbeat, p);
	default:
		return -ENOTTY;
	}
}


static int wdtpci_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &open_lock))
		return -EBUSY;

	if (nowayout)
		__module_get(THIS_MODULE);
	/*
	 *	Activate
	 */
	wdtpci_start();
	return nonseekable_open(inode, file);
}


static int wdtpci_release(struct inode *inode, struct file *file)
{
	if (expect_close == 42) {
		wdtpci_stop();
	} else {
		printk(KERN_CRIT PFX "Unexpected close, not stopping timer!");
		wdtpci_ping();
	}
	expect_close = 0;
	clear_bit(0, &open_lock);
	return 0;
}


static ssize_t wdtpci_temp_read(struct file *file, char __user *buf,
						size_t count, loff_t *ptr)
{
	int temperature;

	if (wdtpci_get_temperature(&temperature))
		return -EFAULT;

	if (copy_to_user(buf, &temperature, 1))
		return -EFAULT;

	return 1;
}


static int wdtpci_temp_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}


static int wdtpci_temp_release(struct inode *inode, struct file *file)
{
	return 0;
}


static int wdtpci_notify_sys(struct notifier_block *this, unsigned long code,
							void *unused)
{
	if (code == SYS_DOWN || code == SYS_HALT)
		wdtpci_stop();
	return NOTIFY_DONE;
}



static const struct file_operations wdtpci_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= wdtpci_write,
	.unlocked_ioctl	= wdtpci_ioctl,
	.open		= wdtpci_open,
	.release	= wdtpci_release,
};

static struct miscdevice wdtpci_miscdev = {
	.minor	= WATCHDOG_MINOR,
	.name	= "watchdog",
	.fops	= &wdtpci_fops,
};

static const struct file_operations wdtpci_temp_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= wdtpci_temp_read,
	.open		= wdtpci_temp_open,
	.release	= wdtpci_temp_release,
};

static struct miscdevice temp_miscdev = {
	.minor	= TEMP_MINOR,
	.name	= "temperature",
	.fops	= &wdtpci_temp_fops,
};


static struct notifier_block wdtpci_notifier = {
	.notifier_call = wdtpci_notify_sys,
};


static int __devinit wdtpci_init_one(struct pci_dev *dev,
					const struct pci_device_id *ent)
{
	int ret = -EIO;

	dev_count++;
	if (dev_count > 1) {
		printk(KERN_ERR PFX "This driver only supports one device\n");
		return -ENODEV;
	}

	if (type != 500 && type != 501) {
		printk(KERN_ERR PFX "unknown card type '%d'.\n", type);
		return -ENODEV;
	}

	if (pci_enable_device(dev)) {
		printk(KERN_ERR PFX "Not possible to enable PCI Device\n");
		return -ENODEV;
	}

	if (pci_resource_start(dev, 2) == 0x0000) {
		printk(KERN_ERR PFX "No I/O-Address for card detected\n");
		ret = -ENODEV;
		goto out_pci;
	}

	if (pci_request_region(dev, 2, "wdt_pci")) {
		printk(KERN_ERR PFX "I/O address 0x%llx already in use\n",
			(unsigned long long)pci_resource_start(dev, 2));
		goto out_pci;
	}

	irq = dev->irq;
	io = pci_resource_start(dev, 2);

	if (request_irq(irq, wdtpci_interrupt, IRQF_DISABLED | IRQF_SHARED,
			 "wdt_pci", &wdtpci_miscdev)) {
		printk(KERN_ERR PFX "IRQ %d is not free\n", irq);
		goto out_reg;
	}

	printk(KERN_INFO
	 "PCI-WDT500/501 (PCI-WDG-CSM) driver 0.10 at 0x%llx (Interrupt %d)\n",
					(unsigned long long)io, irq);

	/* Check that the heartbeat value is within its range;
	   if not reset to the default */
	if (wdtpci_set_heartbeat(heartbeat)) {
		wdtpci_set_heartbeat(WD_TIMO);
		printk(KERN_INFO PFX
		  "heartbeat value must be 0 < heartbeat < 65536, using %d\n",
								WD_TIMO);
	}

	ret = register_reboot_notifier(&wdtpci_notifier);
	if (ret) {
		printk(KERN_ERR PFX
			"cannot register reboot notifier (err=%d)\n", ret);
		goto out_irq;
	}

	if (type == 501) {
		ret = misc_register(&temp_miscdev);
		if (ret) {
			printk(KERN_ERR PFX
			"cannot register miscdev on minor=%d (err=%d)\n",
							TEMP_MINOR, ret);
			goto out_rbt;
		}
	}

	ret = misc_register(&wdtpci_miscdev);
	if (ret) {
		printk(KERN_ERR PFX
			"cannot register miscdev on minor=%d (err=%d)\n",
						WATCHDOG_MINOR, ret);
		goto out_misc;
	}

	printk(KERN_INFO PFX "initialized. heartbeat=%d sec (nowayout=%d)\n",
		heartbeat, nowayout);
	if (type == 501)
		printk(KERN_INFO "wdt: Fan Tachometer is %s\n",
				(tachometer ? "Enabled" : "Disabled"));

	ret = 0;
out:
	return ret;

out_misc:
	if (type == 501)
		misc_deregister(&temp_miscdev);
out_rbt:
	unregister_reboot_notifier(&wdtpci_notifier);
out_irq:
	free_irq(irq, &wdtpci_miscdev);
out_reg:
	pci_release_region(dev, 2);
out_pci:
	pci_disable_device(dev);
	goto out;
}


static void __devexit wdtpci_remove_one(struct pci_dev *pdev)
{
	/* here we assume only one device will ever have
	 * been picked up and registered by probe function */
	misc_deregister(&wdtpci_miscdev);
	if (type == 501)
		misc_deregister(&temp_miscdev);
	unregister_reboot_notifier(&wdtpci_notifier);
	free_irq(irq, &wdtpci_miscdev);
	pci_release_region(pdev, 2);
	pci_disable_device(pdev);
	dev_count--;
}


static struct pci_device_id wdtpci_pci_tbl[] = {
	{
		.vendor	   = PCI_VENDOR_ID_ACCESSIO,
		.device	   = PCI_DEVICE_ID_WDG_CSM,
		.subvendor = PCI_ANY_ID,
		.subdevice = PCI_ANY_ID,
	},
	{ 0, }, /* terminate list */
};
MODULE_DEVICE_TABLE(pci, wdtpci_pci_tbl);


static struct pci_driver wdtpci_driver = {
	.name		= "wdt_pci",
	.id_table	= wdtpci_pci_tbl,
	.probe		= wdtpci_init_one,
	.remove		= __devexit_p(wdtpci_remove_one),
};



static void __exit wdtpci_cleanup(void)
{
	pci_unregister_driver(&wdtpci_driver);
}



static int __init wdtpci_init(void)
{
	return pci_register_driver(&wdtpci_driver);
}


module_init(wdtpci_init);
module_exit(wdtpci_cleanup);

MODULE_AUTHOR("JP Nollmann, Alan Cox");
MODULE_DESCRIPTION("Driver for the ICS PCI-WDT500/501 watchdog cards");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS_MISCDEV(TEMP_MINOR);
