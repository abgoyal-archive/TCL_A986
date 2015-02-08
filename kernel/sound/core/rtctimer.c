

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <linux/log2.h>
#include <sound/core.h>
#include <sound/timer.h>

#if defined(CONFIG_RTC) || defined(CONFIG_RTC_MODULE)

#include <linux/mc146818rtc.h>

#define RTC_FREQ	1024		/* default frequency */
#define NANO_SEC	1000000000L	/* 10^9 in sec */

static int rtctimer_open(struct snd_timer *t);
static int rtctimer_close(struct snd_timer *t);
static int rtctimer_start(struct snd_timer *t);
static int rtctimer_stop(struct snd_timer *t);


static struct snd_timer_hardware rtc_hw = {
	.flags =	SNDRV_TIMER_HW_AUTO |
			SNDRV_TIMER_HW_FIRST |
			SNDRV_TIMER_HW_TASKLET,
	.ticks =	100000000L,		/* FIXME: XXX */
	.open =		rtctimer_open,
	.close =	rtctimer_close,
	.start =	rtctimer_start,
	.stop =		rtctimer_stop,
};

static int rtctimer_freq = RTC_FREQ;		/* frequency */
static struct snd_timer *rtctimer;
static struct tasklet_struct rtc_tasklet;
static rtc_task_t rtc_task;


static int
rtctimer_open(struct snd_timer *t)
{
	int err;

	err = rtc_register(&rtc_task);
	if (err < 0)
		return err;
	t->private_data = &rtc_task;
	return 0;
}

static int
rtctimer_close(struct snd_timer *t)
{
	rtc_task_t *rtc = t->private_data;
	if (rtc) {
		rtc_unregister(rtc);
		tasklet_kill(&rtc_tasklet);
		t->private_data = NULL;
	}
	return 0;
}

static int
rtctimer_start(struct snd_timer *timer)
{
	rtc_task_t *rtc = timer->private_data;
	if (snd_BUG_ON(!rtc))
		return -EINVAL;
	rtc_control(rtc, RTC_IRQP_SET, rtctimer_freq);
	rtc_control(rtc, RTC_PIE_ON, 0);
	return 0;
}

static int
rtctimer_stop(struct snd_timer *timer)
{
	rtc_task_t *rtc = timer->private_data;
	if (snd_BUG_ON(!rtc))
		return -EINVAL;
	rtc_control(rtc, RTC_PIE_OFF, 0);
	return 0;
}

static void rtctimer_tasklet(unsigned long data)
{
	snd_timer_interrupt((struct snd_timer *)data, 1);
}

static void rtctimer_interrupt(void *private_data)
{
	tasklet_schedule(private_data);
}


static int __init rtctimer_init(void)
{
	int err;
	struct snd_timer *timer;

	if (rtctimer_freq < 2 || rtctimer_freq > 8192 ||
	    !is_power_of_2(rtctimer_freq)) {
		snd_printk(KERN_ERR "rtctimer: invalid frequency %d\n",
			   rtctimer_freq);
		return -EINVAL;
	}

	/* Create a new timer and set up the fields */
	err = snd_timer_global_new("rtc", SNDRV_TIMER_GLOBAL_RTC, &timer);
	if (err < 0)
		return err;

	timer->module = THIS_MODULE;
	strcpy(timer->name, "RTC timer");
	timer->hw = rtc_hw;
	timer->hw.resolution = NANO_SEC / rtctimer_freq;

	tasklet_init(&rtc_tasklet, rtctimer_tasklet, (unsigned long)timer);

	/* set up RTC callback */
	rtc_task.func = rtctimer_interrupt;
	rtc_task.private_data = &rtc_tasklet;

	err = snd_timer_global_register(timer);
	if (err < 0) {
		snd_timer_global_free(timer);
		return err;
	}
	rtctimer = timer; /* remember this */

	return 0;
}

static void __exit rtctimer_exit(void)
{
	if (rtctimer) {
		snd_timer_global_free(rtctimer);
		rtctimer = NULL;
	}
}


module_init(rtctimer_init)
module_exit(rtctimer_exit)

module_param(rtctimer_freq, int, 0444);
MODULE_PARM_DESC(rtctimer_freq, "timer frequency in Hz");

MODULE_LICENSE("GPL");

MODULE_ALIAS("snd-timer-" __stringify(SNDRV_TIMER_GLOBAL_RTC));

#endif /* CONFIG_RTC || CONFIG_RTC_MODULE */
