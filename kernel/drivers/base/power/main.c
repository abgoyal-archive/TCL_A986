

#include <linux/device.h>
#include <linux/kallsyms.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/resume-trace.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/async.h>
#include <linux/timer.h>
#include <linux/platform_device.h>

#include "../base.h"
#include "power.h"


LIST_HEAD(dpm_list);

static DEFINE_MUTEX(dpm_list_mtx);
static pm_message_t pm_transition;

static void dpm_drv_timeout(unsigned long data);
static DEFINE_TIMER(dpm_drv_wd, dpm_drv_timeout, 0, 0);

static bool transition_started;

void device_pm_init(struct device *dev)
{
	dev->power.status = DPM_ON;
	init_completion(&dev->power.completion);
	complete_all(&dev->power.completion);
	pm_runtime_init(dev);
}

void device_pm_lock(void)
{
	mutex_lock(&dpm_list_mtx);
}

void device_pm_unlock(void)
{
	mutex_unlock(&dpm_list_mtx);
}

void device_pm_add(struct device *dev)
{
	pr_debug("PM: Adding info for %s:%s\n",
		 dev->bus ? dev->bus->name : "No Bus",
		 kobject_name(&dev->kobj));
	mutex_lock(&dpm_list_mtx);
	if (dev->parent) {
		if (dev->parent->power.status >= DPM_SUSPENDING)
			dev_warn(dev, "parent %s should not be sleeping\n",
				 dev_name(dev->parent));
	} else if (transition_started) {
		/*
		 * We refuse to register parentless devices while a PM
		 * transition is in progress in order to avoid leaving them
		 * unhandled down the road
		 */
		dev_WARN(dev, "Parentless device registered during a PM transaction\n");
	}

	list_add_tail(&dev->power.entry, &dpm_list);
	mutex_unlock(&dpm_list_mtx);
}

void device_pm_remove(struct device *dev)
{
	pr_debug("PM: Removing info for %s:%s\n",
		 dev->bus ? dev->bus->name : "No Bus",
		 kobject_name(&dev->kobj));
	complete_all(&dev->power.completion);
	mutex_lock(&dpm_list_mtx);
	list_del_init(&dev->power.entry);
	mutex_unlock(&dpm_list_mtx);
	pm_runtime_remove(dev);
}

void device_pm_move_before(struct device *deva, struct device *devb)
{
	pr_debug("PM: Moving %s:%s before %s:%s\n",
		 deva->bus ? deva->bus->name : "No Bus",
		 kobject_name(&deva->kobj),
		 devb->bus ? devb->bus->name : "No Bus",
		 kobject_name(&devb->kobj));
	/* Delete deva from dpm_list and reinsert before devb. */
	list_move_tail(&deva->power.entry, &devb->power.entry);
}

void device_pm_move_after(struct device *deva, struct device *devb)
{
	pr_debug("PM: Moving %s:%s after %s:%s\n",
		 deva->bus ? deva->bus->name : "No Bus",
		 kobject_name(&deva->kobj),
		 devb->bus ? devb->bus->name : "No Bus",
		 kobject_name(&devb->kobj));
	/* Delete deva from dpm_list and reinsert after devb. */
	list_move(&deva->power.entry, &devb->power.entry);
}

void device_pm_move_last(struct device *dev)
{
	pr_debug("PM: Moving %s:%s to end of list\n",
		 dev->bus ? dev->bus->name : "No Bus",
		 kobject_name(&dev->kobj));
	list_move_tail(&dev->power.entry, &dpm_list);
}

static ktime_t initcall_debug_start(struct device *dev)
{
	ktime_t calltime = ktime_set(0, 0);

	if (initcall_debug) {
		pr_info("calling  %s+ @ %i\n",
				dev_name(dev), task_pid_nr(current));
		calltime = ktime_get();
	}

	return calltime;
}

static void initcall_debug_report(struct device *dev, ktime_t calltime,
				  int error)
{
	ktime_t delta, rettime;

	if (initcall_debug) {
		rettime = ktime_get();
		delta = ktime_sub(rettime, calltime);
		pr_info("call %s+ returned %d after %Ld usecs\n", dev_name(dev),
			error, (unsigned long long)ktime_to_ns(delta) >> 10);
	}
}

static void dpm_wait(struct device *dev, bool async)
{
	if (!dev)
		return;

	if (async || (pm_async_enabled && dev->power.async_suspend))
		wait_for_completion(&dev->power.completion);
}

static int dpm_wait_fn(struct device *dev, void *async_ptr)
{
	dpm_wait(dev, *((bool *)async_ptr));
	return 0;
}

static void dpm_wait_for_children(struct device *dev, bool async)
{
       device_for_each_child(dev, &async, dpm_wait_fn);
}

static int pm_op(struct device *dev,
		 const struct dev_pm_ops *ops,
		 pm_message_t state)
{
	int error = 0;
	ktime_t calltime;

	calltime = initcall_debug_start(dev);

	switch (state.event) {
#ifdef CONFIG_SUSPEND
	case PM_EVENT_SUSPEND:
		if (ops->suspend) {
			error = ops->suspend(dev);
			suspend_report_result(ops->suspend, error);
		}
		break;
	case PM_EVENT_RESUME:
		if (ops->resume) {
			error = ops->resume(dev);
			suspend_report_result(ops->resume, error);
		}
		break;
#endif /* CONFIG_SUSPEND */
#ifdef CONFIG_HIBERNATION
	case PM_EVENT_FREEZE:
	case PM_EVENT_QUIESCE:
		if (ops->freeze) {
			error = ops->freeze(dev);
			suspend_report_result(ops->freeze, error);
		}
		break;
	case PM_EVENT_HIBERNATE:
		if (ops->poweroff) {
			error = ops->poweroff(dev);
			suspend_report_result(ops->poweroff, error);
		}
		break;
	case PM_EVENT_THAW:
	case PM_EVENT_RECOVER:
		if (ops->thaw) {
			error = ops->thaw(dev);
			suspend_report_result(ops->thaw, error);
		}
		break;
	case PM_EVENT_RESTORE:
		if (ops->restore) {
			error = ops->restore(dev);
			suspend_report_result(ops->restore, error);
		}
		break;
#endif /* CONFIG_HIBERNATION */
	default:
		error = -EINVAL;
	}

	initcall_debug_report(dev, calltime, error);

	return error;
}

static int pm_noirq_op(struct device *dev,
			const struct dev_pm_ops *ops,
			pm_message_t state)
{
	int error = 0;
	ktime_t calltime, delta, rettime;

	if (initcall_debug) {
		pr_info("calling  %s+ @ %i, parent: %s\n",
				dev_name(dev), task_pid_nr(current),
				dev->parent ? dev_name(dev->parent) : "none");
		calltime = ktime_get();
	}

	switch (state.event) {
#ifdef CONFIG_SUSPEND
	case PM_EVENT_SUSPEND:
		if (ops->suspend_noirq) {
			error = ops->suspend_noirq(dev);
			suspend_report_result(ops->suspend_noirq, error);
		}
		break;
	case PM_EVENT_RESUME:
		if (ops->resume_noirq) {
			error = ops->resume_noirq(dev);
			suspend_report_result(ops->resume_noirq, error);
		}
		break;
#endif /* CONFIG_SUSPEND */
#ifdef CONFIG_HIBERNATION
	case PM_EVENT_FREEZE:
	case PM_EVENT_QUIESCE:
		if (ops->freeze_noirq) {
			error = ops->freeze_noirq(dev);
			suspend_report_result(ops->freeze_noirq, error);
		}
		break;
	case PM_EVENT_HIBERNATE:
		if (ops->poweroff_noirq) {
			error = ops->poweroff_noirq(dev);
			suspend_report_result(ops->poweroff_noirq, error);
		}
		break;
	case PM_EVENT_THAW:
	case PM_EVENT_RECOVER:
		if (ops->thaw_noirq) {
			error = ops->thaw_noirq(dev);
			suspend_report_result(ops->thaw_noirq, error);
		}
		break;
	case PM_EVENT_RESTORE:
		if (ops->restore_noirq) {
			error = ops->restore_noirq(dev);
			suspend_report_result(ops->restore_noirq, error);
		}
		break;
#endif /* CONFIG_HIBERNATION */
	default:
		error = -EINVAL;
	}

	if (initcall_debug) {
		rettime = ktime_get();
		delta = ktime_sub(rettime, calltime);
		printk("initcall %s_i+ returned %d after %Ld usecs\n",
			dev_name(dev), error,
			(unsigned long long)ktime_to_ns(delta) >> 10);
	}

	return error;
}

static char *pm_verb(int event)
{
	switch (event) {
	case PM_EVENT_SUSPEND:
		return "suspend";
	case PM_EVENT_RESUME:
		return "resume";
	case PM_EVENT_FREEZE:
		return "freeze";
	case PM_EVENT_QUIESCE:
		return "quiesce";
	case PM_EVENT_HIBERNATE:
		return "hibernate";
	case PM_EVENT_THAW:
		return "thaw";
	case PM_EVENT_RESTORE:
		return "restore";
	case PM_EVENT_RECOVER:
		return "recover";
	default:
		return "(unknown PM event)";
	}
}

static void pm_dev_dbg(struct device *dev, pm_message_t state, char *info)
{
	dev_dbg(dev, "%s%s%s\n", info, pm_verb(state.event),
		((state.event & PM_EVENT_SLEEP) && device_may_wakeup(dev)) ?
		", may wakeup" : "");
}

static void pm_dev_err(struct device *dev, pm_message_t state, char *info,
			int error)
{
	printk(KERN_ERR "PM: Device %s failed to %s%s: error %d\n",
		kobject_name(&dev->kobj), pm_verb(state.event), info, error);
}

static void dpm_show_time(ktime_t starttime, pm_message_t state, char *info)
{
	ktime_t calltime;
	s64 usecs64;
	int usecs;

	calltime = ktime_get();
	usecs64 = ktime_to_ns(ktime_sub(calltime, starttime));
	do_div(usecs64, NSEC_PER_USEC);
	usecs = usecs64;
	if (usecs == 0)
		usecs = 1;
	pr_info("PM: %s%s%s of devices complete after %ld.%03ld msecs\n",
		info ?: "", info ? " " : "", pm_verb(state.event),
		usecs / USEC_PER_MSEC, usecs % USEC_PER_MSEC);
}

/*------------------------- Resume routines -------------------------*/

static int device_resume_noirq(struct device *dev, pm_message_t state)
{
	int error = 0;

	TRACE_DEVICE(dev);
	TRACE_RESUME(0);

	if (dev->bus && dev->bus->pm) {
		pm_dev_dbg(dev, state, "EARLY ");
		error = pm_noirq_op(dev, dev->bus->pm, state);
		if (error)
			goto End;
	}

	if (dev->type && dev->type->pm) {
		pm_dev_dbg(dev, state, "EARLY type ");
		error = pm_noirq_op(dev, dev->type->pm, state);
		if (error)
			goto End;
	}

	if (dev->class && dev->class->pm) {
		pm_dev_dbg(dev, state, "EARLY class ");
		error = pm_noirq_op(dev, dev->class->pm, state);
	}

End:
	TRACE_RESUME(error);
	return error;
}

void dpm_resume_noirq(pm_message_t state)
{
	struct device *dev;
	ktime_t starttime = ktime_get();

	mutex_lock(&dpm_list_mtx);
	transition_started = false;
	list_for_each_entry(dev, &dpm_list, power.entry)
		if (dev->power.status > DPM_OFF) {
			int error;

			dev->power.status = DPM_OFF;
			error = device_resume_noirq(dev, state);
			if (error)
				pm_dev_err(dev, state, " early", error);
		}
	mutex_unlock(&dpm_list_mtx);
	dpm_show_time(starttime, state, "early");
	resume_device_irqs();
}
EXPORT_SYMBOL_GPL(dpm_resume_noirq);

static int legacy_resume(struct device *dev, int (*cb)(struct device *dev))
{
	int error;
	ktime_t calltime;

	calltime = initcall_debug_start(dev);

	error = cb(dev);
	suspend_report_result(cb, error);

	initcall_debug_report(dev, calltime, error);

	return error;
}

static int device_suspend_index = 0; 
static int device_resume_index = 0; 

static int device_resume(struct device *dev, pm_message_t state, bool async)
{
	int error = 0;
	//struct platform_device	*pdev = to_platform_device(dev);

	TRACE_DEVICE(dev);
	TRACE_RESUME(0);

	if (dev->parent && dev->parent->power.status >= DPM_OFF)
		dpm_wait(dev->parent, async);
	device_lock(dev);

	dev->power.status = DPM_RESUMING;

	if (dev->bus) {
		if (dev->bus->pm) {
			pm_dev_dbg(dev, state, "");
			/*if ((u32)pdev->name & 0xC0000000)
				printk("[%d][%s] device_resume\n\r",device_resume_index, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] device_resume\r\n",device_resume_index);
			error = pm_op(dev, dev->bus->pm, state);
			/*if ((u32)pdev->name & 0xC0000000)
				printk("[%d][%s] device_resume pass\n\r",device_resume_index++, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] device_resume pass\r\n",device_resume_index++);
		} else if (dev->bus->resume) {
			pm_dev_dbg(dev, state, "legacy ");
			/*if (pdev->name)
				printk("[%d][%s] legacy device_resume\n\r",device_resume_index, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] legacy device_resume\r\n",device_resume_index);
			error = legacy_resume(dev, dev->bus->resume);
			/*if (pdev->name)
				printk("[%d][%s] legacy device_resume pass\n\r",device_resume_index++, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] legacy device_resume pass\r\n",device_resume_index++);
		}
		if (error)
			goto End;
	}

	if (dev->type) {
		if (dev->type->pm) {
			pm_dev_dbg(dev, state, "type ");
			/*if (pdev->name)
				printk("[%d][%s] type device_resume\n\r",device_resume_index, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] type device_resume\r\n",device_resume_index);
			error = pm_op(dev, dev->type->pm, state);
			/*if (pdev->name)
				printk("[%d][%s] type device_resume pass\n\r",device_resume_index++, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] type device_resume pass\r\n",device_resume_index++);
		}
		if (error)
			goto End;
	}

	if (dev->class) {
		if (dev->class->pm) {
			pm_dev_dbg(dev, state, "class ");
			/*if (pdev->name)
				printk("[%d][%s] class device_resume\n\r",device_resume_index, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] class device_resume\r\n",device_resume_index);
			error = pm_op(dev, dev->class->pm, state);
			/*if (pdev->name)
				printk("[%d][%s] class device_resume pass\n\r",device_resume_index++, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] class device_resume pass\r\n",device_resume_index++);
		} else if (dev->class->resume) {
			pm_dev_dbg(dev, state, "legacy class ");
			/*if (pdev->name)
				printk("[%d][%s] legacy class device_resume\n\r",device_resume_index, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] legacy class device_resume\r\n",device_resume_index);
			error = legacy_resume(dev, dev->class->resume);
			/*if (pdev->name)
				printk("[%d][%s] legacy class device_resume pass\n\r",device_resume_index++, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] legacy class device_resume pass\r\n",device_resume_index++);
		}
	}
 End:
	device_unlock(dev);
	complete_all(&dev->power.completion);

	TRACE_RESUME(error);
	return error;
}

static void async_resume(void *data, async_cookie_t cookie)
{
	struct device *dev = (struct device *)data;
	int error;

	error = device_resume(dev, pm_transition, true);
	if (error)
		pm_dev_err(dev, pm_transition, " async", error);
	put_device(dev);
}

static bool is_async(struct device *dev)
{
	return dev->power.async_suspend && pm_async_enabled
		&& !pm_trace_is_enabled();
}

static void dpm_drv_timeout(unsigned long data)
{
	struct device *dev = (struct device *) data;

	printk(KERN_EMERG "**** DPM device timeout: %s (%s)\n", dev_name(dev),
	       (dev->driver ? dev->driver->name : "no driver"));
        if(strstr(dev_name(dev), "mmc") != NULL)
            return;
	BUG();
}

static void dpm_drv_wdset(struct device *dev)
{
	dpm_drv_wd.data = (unsigned long) dev;
	mod_timer(&dpm_drv_wd, jiffies + (HZ * 3));
}

static void dpm_drv_wdclr(struct device *dev)
{
	del_timer_sync(&dpm_drv_wd);
}

static void dpm_resume(pm_message_t state)
{
	struct list_head list;
	struct device *dev;
	ktime_t starttime = ktime_get();

	INIT_LIST_HEAD(&list);
	mutex_lock(&dpm_list_mtx);
	pm_transition = state;

	list_for_each_entry(dev, &dpm_list, power.entry) {
		if (dev->power.status < DPM_OFF)
			continue;

		INIT_COMPLETION(dev->power.completion);
		if (is_async(dev)) {
			get_device(dev);
			async_schedule(async_resume, dev);
		}
	}

	while (!list_empty(&dpm_list)) {
		dev = to_device(dpm_list.next);
		get_device(dev);
		if (dev->power.status >= DPM_OFF && !is_async(dev)) {
			int error;

			mutex_unlock(&dpm_list_mtx);

			error = device_resume(dev, state, false);

			mutex_lock(&dpm_list_mtx);
			if (error)
				pm_dev_err(dev, state, "", error);
		} else if (dev->power.status == DPM_SUSPENDING) {
			/* Allow new children of the device to be registered */
			dev->power.status = DPM_RESUMING;
		}
		if (!list_empty(&dev->power.entry))
			list_move_tail(&dev->power.entry, &list);
		put_device(dev);
	}
	list_splice(&list, &dpm_list);
	mutex_unlock(&dpm_list_mtx);
	async_synchronize_full();
	dpm_show_time(starttime, state, NULL);
}

static void device_complete(struct device *dev, pm_message_t state)
{
	device_lock(dev);

	if (dev->class && dev->class->pm && dev->class->pm->complete) {
		pm_dev_dbg(dev, state, "completing class ");
		dev->class->pm->complete(dev);
	}

	if (dev->type && dev->type->pm && dev->type->pm->complete) {
		pm_dev_dbg(dev, state, "completing type ");
		dev->type->pm->complete(dev);
	}

	if (dev->bus && dev->bus->pm && dev->bus->pm->complete) {
		pm_dev_dbg(dev, state, "completing ");
		dev->bus->pm->complete(dev);
	}

	device_unlock(dev);
}

static void dpm_complete(pm_message_t state)
{
	struct list_head list;

	INIT_LIST_HEAD(&list);
	mutex_lock(&dpm_list_mtx);
	transition_started = false;
	while (!list_empty(&dpm_list)) {
		struct device *dev = to_device(dpm_list.prev);

		get_device(dev);
		if (dev->power.status > DPM_ON) {
			dev->power.status = DPM_ON;
			mutex_unlock(&dpm_list_mtx);

			device_complete(dev, state);
			pm_runtime_put_sync(dev);

			mutex_lock(&dpm_list_mtx);
		}
		if (!list_empty(&dev->power.entry))
			list_move(&dev->power.entry, &list);
		put_device(dev);
	}
	list_splice(&list, &dpm_list);
	mutex_unlock(&dpm_list_mtx);
}

void dpm_resume_end(pm_message_t state)
{
	might_sleep();
	dpm_resume(state);
	dpm_complete(state);
}
EXPORT_SYMBOL_GPL(dpm_resume_end);


/*------------------------- Suspend routines -------------------------*/

static pm_message_t resume_event(pm_message_t sleep_state)
{
	switch (sleep_state.event) {
	case PM_EVENT_SUSPEND:
		return PMSG_RESUME;
	case PM_EVENT_FREEZE:
	case PM_EVENT_QUIESCE:
		return PMSG_RECOVER;
	case PM_EVENT_HIBERNATE:
		return PMSG_RESTORE;
	}
	return PMSG_ON;
}

static int device_suspend_noirq(struct device *dev, pm_message_t state)
{
	int error = 0;

	if (dev->class && dev->class->pm) {
		pm_dev_dbg(dev, state, "LATE class ");
		error = pm_noirq_op(dev, dev->class->pm, state);
		if (error)
			goto End;
	}

	if (dev->type && dev->type->pm) {
		pm_dev_dbg(dev, state, "LATE type ");
		error = pm_noirq_op(dev, dev->type->pm, state);
		if (error)
			goto End;
	}

	if (dev->bus && dev->bus->pm) {
		pm_dev_dbg(dev, state, "LATE ");
		error = pm_noirq_op(dev, dev->bus->pm, state);
	}

End:
	return error;
}

int dpm_suspend_noirq(pm_message_t state)
{
	struct device *dev;
	ktime_t starttime = ktime_get();
	int error = 0;

	suspend_device_irqs();
	mutex_lock(&dpm_list_mtx);
	list_for_each_entry_reverse(dev, &dpm_list, power.entry) {
		error = device_suspend_noirq(dev, state);
		if (error) {
			pm_dev_err(dev, state, " late", error);
			break;
		}
		dev->power.status = DPM_OFF_IRQ;
	}
	mutex_unlock(&dpm_list_mtx);
	if (error)
		dpm_resume_noirq(resume_event(state));
	else
		dpm_show_time(starttime, state, "late");
	return error;
}
EXPORT_SYMBOL_GPL(dpm_suspend_noirq);

static int legacy_suspend(struct device *dev, pm_message_t state,
			  int (*cb)(struct device *dev, pm_message_t state))
{
	int error;
	ktime_t calltime;

	calltime = initcall_debug_start(dev);

	error = cb(dev, state);
	suspend_report_result(cb, error);

	initcall_debug_report(dev, calltime, error);

	return error;
}

static int async_error;

static int __device_suspend(struct device *dev, pm_message_t state, bool async)
{
	int error = 0;

	//struct platform_device	*pdev = to_platform_device(dev);

	dpm_wait_for_children(dev, async);
	device_lock(dev);

	if (async_error)
		goto End;

	if (dev->class) {
		if (dev->class->pm) {
			pm_dev_dbg(dev, state, "class ");
			/*if (pdev->name)
				printk("[%d][%s] class device_suspend\n\r",device_suspend_index, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] class device_suspend\r\n",device_suspend_index);
			error = pm_op(dev, dev->class->pm, state);
			/*if (pdev->name)
				printk("[%d][%s] class device_suspend pass\n\r",device_suspend_index++, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] class device_suspend pass\r\n",device_suspend_index++);
		} else if (dev->class->suspend) {
			pm_dev_dbg(dev, state, "legacy class ");
			/*if (pdev->name)
				printk("[%d][%s] legacy class device_suspend\n\r",device_suspend_index, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] legacy class device_suspend\r\n",device_suspend_index);
			error = legacy_suspend(dev, state, dev->class->suspend);
			/*if (pdev->name)
				printk("[%d][%s] legacy class device_suspend pass\n\r",device_suspend_index++, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] legacy class device_suspend pass\r\n",device_suspend_index++);
			suspend_report_result(dev->class->suspend, error);
		}
		if (error)
			goto End;
	}

	if (dev->type) {
		if (dev->type->pm) {
			pm_dev_dbg(dev, state, "type ");
			/*if (pdev->name)
				printk("[%d][%s] type device_suspend\n\r",device_suspend_index, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] type device_suspend\r\n",device_suspend_index);
			error = pm_op(dev, dev->type->pm, state);
			/*if (pdev->name)
				printk("[%d][%s] type device_suspend pass\n\r",device_suspend_index++, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] type device_suspend pass\r\n",device_suspend_index++);
		}
		if (error)
			goto End;
	}

	if (dev->bus) {
		if (dev->bus->pm) {
			pm_dev_dbg(dev, state, "");
			/*if ((u32)pdev->name & 0xC0000000)
				printk("[%d][%s] device_suspend\n\r",device_suspend_index, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] device_suspend\r\n",device_suspend_index);
			error = pm_op(dev, dev->bus->pm, state);
			/*if ((u32)pdev->name & 0xC0000000)
				printk("[%d][%s] device_suspend pass\n\r",device_suspend_index++, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] device_suspend pass\r\n",device_suspend_index++);
		} else if (dev->bus->suspend) {
			pm_dev_dbg(dev, state, "legacy ");
			/*if (pdev->name)
				printk("[%d][%s] legacy device_suspend\n\r",device_suspend_index, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] legacy device_suspend\r\n",device_suspend_index);
			error = legacy_suspend(dev, state, dev->bus->suspend);
			/*if (pdev->name)
				printk("[%d][%s] legacy device_suspend pass\n\r",device_suspend_index++, pdev->name);
			else*/
				printk(KERN_DEBUG "[%d] legacy device_suspend pass\r\n",device_suspend_index++);
			suspend_report_result(dev->bus->suspend, error);
		}
	}

	if (!error)
		dev->power.status = DPM_OFF;

 End:
	device_unlock(dev);
	complete_all(&dev->power.completion);

	return error;
}

static void async_suspend(void *data, async_cookie_t cookie)
{
	struct device *dev = (struct device *)data;
	int error;

	error = __device_suspend(dev, pm_transition, true);
	if (error) {
		pm_dev_err(dev, pm_transition, " async", error);
		async_error = error;
	}

	put_device(dev);
}

static int device_suspend(struct device *dev)
{
	INIT_COMPLETION(dev->power.completion);

	if (pm_async_enabled && dev->power.async_suspend) {
		get_device(dev);
		async_schedule(async_suspend, dev);
		return 0;
	}

	return __device_suspend(dev, pm_transition, false);
}

static int dpm_suspend(pm_message_t state)
{
	struct list_head list;
	ktime_t starttime = ktime_get();
	int error = 0;

	INIT_LIST_HEAD(&list);
	mutex_lock(&dpm_list_mtx);
	pm_transition = state;
	async_error = 0;
	while (!list_empty(&dpm_list)) {
		struct device *dev = to_device(dpm_list.prev);

		get_device(dev);
		mutex_unlock(&dpm_list_mtx);

		dpm_drv_wdset(dev);
		error = device_suspend(dev);
		dpm_drv_wdclr(dev);

		mutex_lock(&dpm_list_mtx);
		if (error) {
			pm_dev_err(dev, state, "", error);
			put_device(dev);
			break;
		}
		if (!list_empty(&dev->power.entry))
			list_move(&dev->power.entry, &list);
		put_device(dev);
		if (async_error)
			break;
	}
	list_splice(&list, dpm_list.prev);
	mutex_unlock(&dpm_list_mtx);
	async_synchronize_full();
	if (!error)
		error = async_error;
	if (!error)
		dpm_show_time(starttime, state, NULL);
	return error;
}

static int device_prepare(struct device *dev, pm_message_t state)
{
	int error = 0;

	device_lock(dev);

	if (dev->bus && dev->bus->pm && dev->bus->pm->prepare) {
		pm_dev_dbg(dev, state, "preparing ");
		error = dev->bus->pm->prepare(dev);
		suspend_report_result(dev->bus->pm->prepare, error);
		if (error)
			goto End;
	}

	if (dev->type && dev->type->pm && dev->type->pm->prepare) {
		pm_dev_dbg(dev, state, "preparing type ");
		error = dev->type->pm->prepare(dev);
		suspend_report_result(dev->type->pm->prepare, error);
		if (error)
			goto End;
	}

	if (dev->class && dev->class->pm && dev->class->pm->prepare) {
		pm_dev_dbg(dev, state, "preparing class ");
		error = dev->class->pm->prepare(dev);
		suspend_report_result(dev->class->pm->prepare, error);
	}
 End:
	device_unlock(dev);

	return error;
}

static int dpm_prepare(pm_message_t state)
{
	struct list_head list;
	int error = 0;

	INIT_LIST_HEAD(&list);
	mutex_lock(&dpm_list_mtx);
	transition_started = true;
	while (!list_empty(&dpm_list)) {
		struct device *dev = to_device(dpm_list.next);

		get_device(dev);
		dev->power.status = DPM_PREPARING;
		mutex_unlock(&dpm_list_mtx);

		pm_runtime_get_noresume(dev);
		if (pm_runtime_barrier(dev) && device_may_wakeup(dev)) {
			/* Wake-up requested during system sleep transition. */
			pm_runtime_put_sync(dev);
			error = -EBUSY;
		} else {
			error = device_prepare(dev, state);
		}

		mutex_lock(&dpm_list_mtx);
		if (error) {
			dev->power.status = DPM_ON;
			if (error == -EAGAIN) {
				put_device(dev);
				error = 0;
				continue;
			}
			printk(KERN_ERR "PM: Failed to prepare device %s "
				"for power transition: error %d\n",
				kobject_name(&dev->kobj), error);
			put_device(dev);
			break;
		}
		dev->power.status = DPM_SUSPENDING;
		if (!list_empty(&dev->power.entry))
			list_move_tail(&dev->power.entry, &list);
		put_device(dev);
	}
	list_splice(&list, &dpm_list);
	mutex_unlock(&dpm_list_mtx);
	return error;
}

int dpm_suspend_start(pm_message_t state)
{
	int error;

	might_sleep();
	error = dpm_prepare(state);
	if (!error)
		error = dpm_suspend(state);
	return error;
}
EXPORT_SYMBOL_GPL(dpm_suspend_start);

void __suspend_report_result(const char *function, void *fn, int ret)
{
	if (ret)
		printk(KERN_ERR "%s(): %pF returns %d\n", function, fn, ret);
}
EXPORT_SYMBOL_GPL(__suspend_report_result);

void device_pm_wait_for_dev(struct device *subordinate, struct device *dev)
{
	dpm_wait(dev, subordinate->power.async_suspend);
}
EXPORT_SYMBOL_GPL(device_pm_wait_for_dev);
