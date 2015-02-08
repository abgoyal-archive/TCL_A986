

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>

#include <mach/hardware.h>
#include <mach/jornada720.h>
#include <asm/hardware/ssp.h>

static DEFINE_SPINLOCK(jornada_ssp_lock);
static unsigned long jornada_ssp_flags;

u8 inline jornada_ssp_reverse(u8 byte)
{
	return
		((0x80 & byte) >> 7) |
		((0x40 & byte) >> 5) |
		((0x20 & byte) >> 3) |
		((0x10 & byte) >> 1) |
		((0x08 & byte) << 1) |
		((0x04 & byte) << 3) |
		((0x02 & byte) << 5) |
		((0x01 & byte) << 7);
};
EXPORT_SYMBOL(jornada_ssp_reverse);

int jornada_ssp_byte(u8 byte)
{
	int timeout = 400000;
	u16 ret;

	while ((GPLR & GPIO_GPIO10)) {
		if (!--timeout) {
			printk(KERN_WARNING "SSP: timeout while waiting for transmit\n");
			return -ETIMEDOUT;
		}
		cpu_relax();
	}

	ret = jornada_ssp_reverse(byte) << 8;

	ssp_write_word(ret);
	ssp_read_word(&ret);

	return jornada_ssp_reverse(ret);
};
EXPORT_SYMBOL(jornada_ssp_byte);

int jornada_ssp_inout(u8 byte)
{
	int ret, i;

	/* true means command byte */
	if (byte != TXDUMMY) {
		ret = jornada_ssp_byte(byte);
		/* Proper return to commands is TxDummy */
		if (ret != TXDUMMY) {
			for (i = 0; i < 256; i++)/* flushing bus */
				if (jornada_ssp_byte(TXDUMMY) == -1)
					break;
			return -ETIMEDOUT;
		}
	} else /* Exchange TxDummy for data */
		ret = jornada_ssp_byte(TXDUMMY);

	return ret;
};
EXPORT_SYMBOL(jornada_ssp_inout);

void jornada_ssp_start(void)
{
	spin_lock_irqsave(&jornada_ssp_lock, jornada_ssp_flags);
	GPCR = GPIO_GPIO25;
	udelay(50);
	return;
};
EXPORT_SYMBOL(jornada_ssp_start);

void jornada_ssp_end(void)
{
	GPSR = GPIO_GPIO25;
	spin_unlock_irqrestore(&jornada_ssp_lock, jornada_ssp_flags);
	return;
};
EXPORT_SYMBOL(jornada_ssp_end);

static int __devinit jornada_ssp_probe(struct platform_device *dev)
{
	int ret;

	GPSR = GPIO_GPIO25;

	ret = ssp_init();

	/* worked fine, lets not bother with anything else */
	if (!ret) {
		printk(KERN_INFO "SSP: device initialized with irq\n");
		return ret;
	}

	printk(KERN_WARNING "SSP: initialization failed, trying non-irq solution \n");

	/* init of Serial 4 port */
	Ser4MCCR0 = 0;
	Ser4SSCR0 = 0x0387;
	Ser4SSCR1 = 0x18;

	/* clear out any left over data */
	ssp_flush();

	/* enable MCU */
	jornada_ssp_start();

	/* see if return value makes sense */
	ret = jornada_ssp_inout(GETBRIGHTNESS);

	/* seems like it worked, just feed it with TxDummy to get rid of data */
	if (ret == TXDUMMY)
		jornada_ssp_inout(TXDUMMY);

	jornada_ssp_end();

	/* failed, lets just kill everything */
	if (ret == -ETIMEDOUT) {
		printk(KERN_WARNING "SSP: attempts failed, bailing\n");
		ssp_exit();
		return -ENODEV;
	}

	/* all fine */
	printk(KERN_INFO "SSP: device initialized\n");
	return 0;
};

static int jornada_ssp_remove(struct platform_device *dev)
{
	/* Note that this doesnt actually remove the driver, since theres nothing to remove
	 * It just makes sure everything is turned off */
	GPSR = GPIO_GPIO25;
	ssp_exit();
	return 0;
};

struct platform_driver jornadassp_driver = {
	.probe	= jornada_ssp_probe,
	.remove	= jornada_ssp_remove,
	.driver	= {
		.name	= "jornada_ssp",
	},
};

static int __init jornada_ssp_init(void)
{
	return platform_driver_register(&jornadassp_driver);
}
