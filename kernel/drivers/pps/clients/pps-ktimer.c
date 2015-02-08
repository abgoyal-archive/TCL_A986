


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/pps_kernel.h>


static int source;
static struct timer_list ktimer;


static void pps_ktimer_event(unsigned long ptr)
{
	struct timespec __ts;
	struct pps_ktime ts;

	/* First of all we get the time stamp... */
	getnstimeofday(&__ts);

	pr_info("PPS event at %lu\n", jiffies);

	/* ... and translate it to PPS time data struct */
	ts.sec = __ts.tv_sec;
	ts.nsec = __ts.tv_nsec;

	pps_event(source, &ts, PPS_CAPTUREASSERT, NULL);

	mod_timer(&ktimer, jiffies + HZ);
}


static void pps_ktimer_echo(int source, int event, void *data)
{
	pr_info("echo %s %s for source %d\n",
		event & PPS_CAPTUREASSERT ? "assert" : "",
		event & PPS_CAPTURECLEAR ? "clear" : "",
		source);
}


static struct pps_source_info pps_ktimer_info = {
	.name		= "ktimer",
	.path		= "",
	.mode		= PPS_CAPTUREASSERT | PPS_OFFSETASSERT |
			  PPS_ECHOASSERT |
			  PPS_CANWAIT | PPS_TSFMT_TSPEC,
	.echo		= pps_ktimer_echo,
	.owner		= THIS_MODULE,
};


static void __exit pps_ktimer_exit(void)
{
	del_timer_sync(&ktimer);
	pps_unregister_source(source);

	pr_info("ktimer PPS source unregistered\n");
}

static int __init pps_ktimer_init(void)
{
	int ret;

	ret = pps_register_source(&pps_ktimer_info,
				PPS_CAPTUREASSERT | PPS_OFFSETASSERT);
	if (ret < 0) {
		printk(KERN_ERR "cannot register ktimer source\n");
		return ret;
	}
	source = ret;

	setup_timer(&ktimer, pps_ktimer_event, 0);
	mod_timer(&ktimer, jiffies + HZ);

	pr_info("ktimer PPS source registered at %d\n", source);

	return  0;
}

module_init(pps_ktimer_init);
module_exit(pps_ktimer_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("dummy PPS source by using a kernel timer (just for debug)");
MODULE_LICENSE("GPL");
