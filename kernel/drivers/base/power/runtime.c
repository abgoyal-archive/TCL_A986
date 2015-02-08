

#include <linux/sched.h>
#include <linux/pm_runtime.h>
#include <linux/jiffies.h>

static int __pm_runtime_resume(struct device *dev, bool from_wq);
static int __pm_request_idle(struct device *dev);
static int __pm_request_resume(struct device *dev);

static void pm_runtime_deactivate_timer(struct device *dev)
{
	if (dev->power.timer_expires > 0) {
		del_timer(&dev->power.suspend_timer);
		dev->power.timer_expires = 0;
	}
}

static void pm_runtime_cancel_pending(struct device *dev)
{
	pm_runtime_deactivate_timer(dev);
	/*
	 * In case there's a request pending, make sure its work function will
	 * return without doing anything.
	 */
	dev->power.request = RPM_REQ_NONE;
}

static int __pm_runtime_idle(struct device *dev)
	__releases(&dev->power.lock) __acquires(&dev->power.lock)
{
	int retval = 0;

	if (dev->power.runtime_error)
		retval = -EINVAL;
	else if (dev->power.idle_notification)
		retval = -EINPROGRESS;
	else if (atomic_read(&dev->power.usage_count) > 0
	    || dev->power.disable_depth > 0
	    || dev->power.runtime_status != RPM_ACTIVE)
		retval = -EAGAIN;
	else if (!pm_children_suspended(dev))
		retval = -EBUSY;
	if (retval)
		goto out;

	if (dev->power.request_pending) {
		/*
		 * If an idle notification request is pending, cancel it.  Any
		 * other pending request takes precedence over us.
		 */
		if (dev->power.request == RPM_REQ_IDLE) {
			dev->power.request = RPM_REQ_NONE;
		} else if (dev->power.request != RPM_REQ_NONE) {
			retval = -EAGAIN;
			goto out;
		}
	}

	dev->power.idle_notification = true;

	if (dev->bus && dev->bus->pm && dev->bus->pm->runtime_idle) {
		spin_unlock_irq(&dev->power.lock);

		dev->bus->pm->runtime_idle(dev);

		spin_lock_irq(&dev->power.lock);
	} else if (dev->type && dev->type->pm && dev->type->pm->runtime_idle) {
		spin_unlock_irq(&dev->power.lock);

		dev->type->pm->runtime_idle(dev);

		spin_lock_irq(&dev->power.lock);
	} else if (dev->class && dev->class->pm
	    && dev->class->pm->runtime_idle) {
		spin_unlock_irq(&dev->power.lock);

		dev->class->pm->runtime_idle(dev);

		spin_lock_irq(&dev->power.lock);
	}

	dev->power.idle_notification = false;
	wake_up_all(&dev->power.wait_queue);

 out:
	return retval;
}

int pm_runtime_idle(struct device *dev)
{
	int retval;

	spin_lock_irq(&dev->power.lock);
	retval = __pm_runtime_idle(dev);
	spin_unlock_irq(&dev->power.lock);

	return retval;
}
EXPORT_SYMBOL_GPL(pm_runtime_idle);

int __pm_runtime_suspend(struct device *dev, bool from_wq)
	__releases(&dev->power.lock) __acquires(&dev->power.lock)
{
	struct device *parent = NULL;
	bool notify = false;
	int retval = 0;

	dev_dbg(dev, "__pm_runtime_suspend()%s!\n",
		from_wq ? " from workqueue" : "");

 repeat:
	if (dev->power.runtime_error) {
		retval = -EINVAL;
		goto out;
	}

	/* Pending resume requests take precedence over us. */
	if (dev->power.request_pending
	    && dev->power.request == RPM_REQ_RESUME) {
		retval = -EAGAIN;
		goto out;
	}

	/* Other scheduled or pending requests need to be canceled. */
	pm_runtime_cancel_pending(dev);

	if (dev->power.runtime_status == RPM_SUSPENDED)
		retval = 1;
	else if (dev->power.runtime_status == RPM_RESUMING
	    || dev->power.disable_depth > 0
	    || atomic_read(&dev->power.usage_count) > 0)
		retval = -EAGAIN;
	else if (!pm_children_suspended(dev))
		retval = -EBUSY;
	if (retval)
		goto out;

	if (dev->power.runtime_status == RPM_SUSPENDING) {
		DEFINE_WAIT(wait);

		if (from_wq) {
			retval = -EINPROGRESS;
			goto out;
		}

		/* Wait for the other suspend running in parallel with us. */
		for (;;) {
			prepare_to_wait(&dev->power.wait_queue, &wait,
					TASK_UNINTERRUPTIBLE);
			if (dev->power.runtime_status != RPM_SUSPENDING)
				break;

			spin_unlock_irq(&dev->power.lock);

			schedule();

			spin_lock_irq(&dev->power.lock);
		}
		finish_wait(&dev->power.wait_queue, &wait);
		goto repeat;
	}

	dev->power.runtime_status = RPM_SUSPENDING;
	dev->power.deferred_resume = false;

	if (dev->bus && dev->bus->pm && dev->bus->pm->runtime_suspend) {
		spin_unlock_irq(&dev->power.lock);

		retval = dev->bus->pm->runtime_suspend(dev);

		spin_lock_irq(&dev->power.lock);
		dev->power.runtime_error = retval;
	} else if (dev->type && dev->type->pm
	    && dev->type->pm->runtime_suspend) {
		spin_unlock_irq(&dev->power.lock);

		retval = dev->type->pm->runtime_suspend(dev);

		spin_lock_irq(&dev->power.lock);
		dev->power.runtime_error = retval;
	} else if (dev->class && dev->class->pm
	    && dev->class->pm->runtime_suspend) {
		spin_unlock_irq(&dev->power.lock);

		retval = dev->class->pm->runtime_suspend(dev);

		spin_lock_irq(&dev->power.lock);
		dev->power.runtime_error = retval;
	} else {
		retval = -ENOSYS;
	}

	if (retval) {
		dev->power.runtime_status = RPM_ACTIVE;
		if (retval == -EAGAIN || retval == -EBUSY) {
			if (dev->power.timer_expires == 0)
				notify = true;
			dev->power.runtime_error = 0;
		} else {
			pm_runtime_cancel_pending(dev);
		}
	} else {
		dev->power.runtime_status = RPM_SUSPENDED;
		pm_runtime_deactivate_timer(dev);

		if (dev->parent) {
			parent = dev->parent;
			atomic_add_unless(&parent->power.child_count, -1, 0);
		}
	}
	wake_up_all(&dev->power.wait_queue);

	if (dev->power.deferred_resume) {
		__pm_runtime_resume(dev, false);
		retval = -EAGAIN;
		goto out;
	}

	if (notify)
		__pm_runtime_idle(dev);

	if (parent && !parent->power.ignore_children) {
		spin_unlock_irq(&dev->power.lock);

		pm_request_idle(parent);

		spin_lock_irq(&dev->power.lock);
	}

 out:
	dev_dbg(dev, "__pm_runtime_suspend() returns %d!\n", retval);

	return retval;
}

int pm_runtime_suspend(struct device *dev)
{
	int retval;

	spin_lock_irq(&dev->power.lock);
	retval = __pm_runtime_suspend(dev, false);
	spin_unlock_irq(&dev->power.lock);

	return retval;
}
EXPORT_SYMBOL_GPL(pm_runtime_suspend);

int __pm_runtime_resume(struct device *dev, bool from_wq)
	__releases(&dev->power.lock) __acquires(&dev->power.lock)
{
	struct device *parent = NULL;
	int retval = 0;

	dev_dbg(dev, "__pm_runtime_resume()%s!\n",
		from_wq ? " from workqueue" : "");

 repeat:
	if (dev->power.runtime_error) {
		retval = -EINVAL;
		goto out;
	}

	pm_runtime_cancel_pending(dev);

	if (dev->power.runtime_status == RPM_ACTIVE)
		retval = 1;
	else if (dev->power.disable_depth > 0)
		retval = -EAGAIN;
	if (retval)
		goto out;

	if (dev->power.runtime_status == RPM_RESUMING
	    || dev->power.runtime_status == RPM_SUSPENDING) {
		DEFINE_WAIT(wait);

		if (from_wq) {
			if (dev->power.runtime_status == RPM_SUSPENDING)
				dev->power.deferred_resume = true;
			retval = -EINPROGRESS;
			goto out;
		}

		/* Wait for the operation carried out in parallel with us. */
		for (;;) {
			prepare_to_wait(&dev->power.wait_queue, &wait,
					TASK_UNINTERRUPTIBLE);
			if (dev->power.runtime_status != RPM_RESUMING
			    && dev->power.runtime_status != RPM_SUSPENDING)
				break;

			spin_unlock_irq(&dev->power.lock);

			schedule();

			spin_lock_irq(&dev->power.lock);
		}
		finish_wait(&dev->power.wait_queue, &wait);
		goto repeat;
	}

	if (!parent && dev->parent) {
		/*
		 * Increment the parent's resume counter and resume it if
		 * necessary.
		 */
		parent = dev->parent;
		spin_unlock(&dev->power.lock);

		pm_runtime_get_noresume(parent);

		spin_lock(&parent->power.lock);
		/*
		 * We can resume if the parent's run-time PM is disabled or it
		 * is set to ignore children.
		 */
		if (!parent->power.disable_depth
		    && !parent->power.ignore_children) {
			__pm_runtime_resume(parent, false);
			if (parent->power.runtime_status != RPM_ACTIVE)
				retval = -EBUSY;
		}
		spin_unlock(&parent->power.lock);

		spin_lock(&dev->power.lock);
		if (retval)
			goto out;
		goto repeat;
	}

	dev->power.runtime_status = RPM_RESUMING;

	if (dev->bus && dev->bus->pm && dev->bus->pm->runtime_resume) {
		spin_unlock_irq(&dev->power.lock);

		retval = dev->bus->pm->runtime_resume(dev);

		spin_lock_irq(&dev->power.lock);
		dev->power.runtime_error = retval;
	} else if (dev->type && dev->type->pm
	    && dev->type->pm->runtime_resume) {
		spin_unlock_irq(&dev->power.lock);

		retval = dev->type->pm->runtime_resume(dev);

		spin_lock_irq(&dev->power.lock);
		dev->power.runtime_error = retval;
	} else if (dev->class && dev->class->pm
	    && dev->class->pm->runtime_resume) {
		spin_unlock_irq(&dev->power.lock);

		retval = dev->class->pm->runtime_resume(dev);

		spin_lock_irq(&dev->power.lock);
		dev->power.runtime_error = retval;
	} else {
		retval = -ENOSYS;
	}

	if (retval) {
		dev->power.runtime_status = RPM_SUSPENDED;
		pm_runtime_cancel_pending(dev);
	} else {
		dev->power.runtime_status = RPM_ACTIVE;
		if (parent)
			atomic_inc(&parent->power.child_count);
	}
	wake_up_all(&dev->power.wait_queue);

	if (!retval)
		__pm_request_idle(dev);

 out:
	if (parent) {
		spin_unlock_irq(&dev->power.lock);

		pm_runtime_put(parent);

		spin_lock_irq(&dev->power.lock);
	}

	dev_dbg(dev, "__pm_runtime_resume() returns %d!\n", retval);

	return retval;
}

int pm_runtime_resume(struct device *dev)
{
	int retval;

	spin_lock_irq(&dev->power.lock);
	retval = __pm_runtime_resume(dev, false);
	spin_unlock_irq(&dev->power.lock);

	return retval;
}
EXPORT_SYMBOL_GPL(pm_runtime_resume);

static void pm_runtime_work(struct work_struct *work)
{
	struct device *dev = container_of(work, struct device, power.work);
	enum rpm_request req;

	spin_lock_irq(&dev->power.lock);

	if (!dev->power.request_pending)
		goto out;

	req = dev->power.request;
	dev->power.request = RPM_REQ_NONE;
	dev->power.request_pending = false;

	switch (req) {
	case RPM_REQ_NONE:
		break;
	case RPM_REQ_IDLE:
		__pm_runtime_idle(dev);
		break;
	case RPM_REQ_SUSPEND:
		__pm_runtime_suspend(dev, true);
		break;
	case RPM_REQ_RESUME:
		__pm_runtime_resume(dev, true);
		break;
	}

 out:
	spin_unlock_irq(&dev->power.lock);
}

static int __pm_request_idle(struct device *dev)
{
	int retval = 0;

	if (dev->power.runtime_error)
		retval = -EINVAL;
	else if (atomic_read(&dev->power.usage_count) > 0
	    || dev->power.disable_depth > 0
	    || dev->power.runtime_status == RPM_SUSPENDED
	    || dev->power.runtime_status == RPM_SUSPENDING)
		retval = -EAGAIN;
	else if (!pm_children_suspended(dev))
		retval = -EBUSY;
	if (retval)
		return retval;

	if (dev->power.request_pending) {
		/* Any requests other then RPM_REQ_IDLE take precedence. */
		if (dev->power.request == RPM_REQ_NONE)
			dev->power.request = RPM_REQ_IDLE;
		else if (dev->power.request != RPM_REQ_IDLE)
			retval = -EAGAIN;
		return retval;
	}

	dev->power.request = RPM_REQ_IDLE;
	dev->power.request_pending = true;
	queue_work(pm_wq, &dev->power.work);

	return retval;
}

int pm_request_idle(struct device *dev)
{
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&dev->power.lock, flags);
	retval = __pm_request_idle(dev);
	spin_unlock_irqrestore(&dev->power.lock, flags);

	return retval;
}
EXPORT_SYMBOL_GPL(pm_request_idle);

static int __pm_request_suspend(struct device *dev)
{
	int retval = 0;

	if (dev->power.runtime_error)
		return -EINVAL;

	if (dev->power.runtime_status == RPM_SUSPENDED)
		retval = 1;
	else if (atomic_read(&dev->power.usage_count) > 0
	    || dev->power.disable_depth > 0)
		retval = -EAGAIN;
	else if (dev->power.runtime_status == RPM_SUSPENDING)
		retval = -EINPROGRESS;
	else if (!pm_children_suspended(dev))
		retval = -EBUSY;
	if (retval < 0)
		return retval;

	pm_runtime_deactivate_timer(dev);

	if (dev->power.request_pending) {
		/*
		 * Pending resume requests take precedence over us, but we can
		 * overtake any other pending request.
		 */
		if (dev->power.request == RPM_REQ_RESUME)
			retval = -EAGAIN;
		else if (dev->power.request != RPM_REQ_SUSPEND)
			dev->power.request = retval ?
						RPM_REQ_NONE : RPM_REQ_SUSPEND;
		return retval;
	} else if (retval) {
		return retval;
	}

	dev->power.request = RPM_REQ_SUSPEND;
	dev->power.request_pending = true;
	queue_work(pm_wq, &dev->power.work);

	return 0;
}

static void pm_suspend_timer_fn(unsigned long data)
{
	struct device *dev = (struct device *)data;
	unsigned long flags;
	unsigned long expires;

	spin_lock_irqsave(&dev->power.lock, flags);

	expires = dev->power.timer_expires;
	/* If 'expire' is after 'jiffies' we've been called too early. */
	if (expires > 0 && !time_after(expires, jiffies)) {
		dev->power.timer_expires = 0;
		__pm_request_suspend(dev);
	}

	spin_unlock_irqrestore(&dev->power.lock, flags);
}

int pm_schedule_suspend(struct device *dev, unsigned int delay)
{
	unsigned long flags;
	int retval = 0;

	spin_lock_irqsave(&dev->power.lock, flags);

	if (dev->power.runtime_error) {
		retval = -EINVAL;
		goto out;
	}

	if (!delay) {
		retval = __pm_request_suspend(dev);
		goto out;
	}

	pm_runtime_deactivate_timer(dev);

	if (dev->power.request_pending) {
		/*
		 * Pending resume requests take precedence over us, but any
		 * other pending requests have to be canceled.
		 */
		if (dev->power.request == RPM_REQ_RESUME) {
			retval = -EAGAIN;
			goto out;
		}
		dev->power.request = RPM_REQ_NONE;
	}

	if (dev->power.runtime_status == RPM_SUSPENDED)
		retval = 1;
	else if (atomic_read(&dev->power.usage_count) > 0
	    || dev->power.disable_depth > 0)
		retval = -EAGAIN;
	else if (!pm_children_suspended(dev))
		retval = -EBUSY;
	if (retval)
		goto out;

	dev->power.timer_expires = jiffies + msecs_to_jiffies(delay);
	if (!dev->power.timer_expires)
		dev->power.timer_expires = 1;
	mod_timer(&dev->power.suspend_timer, dev->power.timer_expires);

 out:
	spin_unlock_irqrestore(&dev->power.lock, flags);

	return retval;
}
EXPORT_SYMBOL_GPL(pm_schedule_suspend);

static int __pm_request_resume(struct device *dev)
{
	int retval = 0;

	if (dev->power.runtime_error)
		return -EINVAL;

	if (dev->power.runtime_status == RPM_ACTIVE)
		retval = 1;
	else if (dev->power.runtime_status == RPM_RESUMING)
		retval = -EINPROGRESS;
	else if (dev->power.disable_depth > 0)
		retval = -EAGAIN;
	if (retval < 0)
		return retval;

	pm_runtime_deactivate_timer(dev);

	if (dev->power.runtime_status == RPM_SUSPENDING) {
		dev->power.deferred_resume = true;
		return retval;
	}
	if (dev->power.request_pending) {
		/* If non-resume request is pending, we can overtake it. */
		dev->power.request = retval ? RPM_REQ_NONE : RPM_REQ_RESUME;
		return retval;
	}
	if (retval)
		return retval;

	dev->power.request = RPM_REQ_RESUME;
	dev->power.request_pending = true;
	queue_work(pm_wq, &dev->power.work);

	return retval;
}

int pm_request_resume(struct device *dev)
{
	unsigned long flags;
	int retval;

	spin_lock_irqsave(&dev->power.lock, flags);
	retval = __pm_request_resume(dev);
	spin_unlock_irqrestore(&dev->power.lock, flags);

	return retval;
}
EXPORT_SYMBOL_GPL(pm_request_resume);

int __pm_runtime_get(struct device *dev, bool sync)
{
	int retval;

	atomic_inc(&dev->power.usage_count);
	retval = sync ? pm_runtime_resume(dev) : pm_request_resume(dev);

	return retval;
}
EXPORT_SYMBOL_GPL(__pm_runtime_get);

int __pm_runtime_put(struct device *dev, bool sync)
{
	int retval = 0;

	if (atomic_dec_and_test(&dev->power.usage_count))
		retval = sync ? pm_runtime_idle(dev) : pm_request_idle(dev);

	return retval;
}
EXPORT_SYMBOL_GPL(__pm_runtime_put);

int __pm_runtime_set_status(struct device *dev, unsigned int status)
{
	struct device *parent = dev->parent;
	unsigned long flags;
	bool notify_parent = false;
	int error = 0;

	if (status != RPM_ACTIVE && status != RPM_SUSPENDED)
		return -EINVAL;

	spin_lock_irqsave(&dev->power.lock, flags);

	if (!dev->power.runtime_error && !dev->power.disable_depth) {
		error = -EAGAIN;
		goto out;
	}

	if (dev->power.runtime_status == status)
		goto out_set;

	if (status == RPM_SUSPENDED) {
		/* It always is possible to set the status to 'suspended'. */
		if (parent) {
			atomic_add_unless(&parent->power.child_count, -1, 0);
			notify_parent = !parent->power.ignore_children;
		}
		goto out_set;
	}

	if (parent) {
		spin_lock_nested(&parent->power.lock, SINGLE_DEPTH_NESTING);

		/*
		 * It is invalid to put an active child under a parent that is
		 * not active, has run-time PM enabled and the
		 * 'power.ignore_children' flag unset.
		 */
		if (!parent->power.disable_depth
		    && !parent->power.ignore_children
		    && parent->power.runtime_status != RPM_ACTIVE)
			error = -EBUSY;
		else if (dev->power.runtime_status == RPM_SUSPENDED)
			atomic_inc(&parent->power.child_count);

		spin_unlock(&parent->power.lock);

		if (error)
			goto out;
	}

 out_set:
	dev->power.runtime_status = status;
	dev->power.runtime_error = 0;
 out:
	spin_unlock_irqrestore(&dev->power.lock, flags);

	if (notify_parent)
		pm_request_idle(parent);

	return error;
}
EXPORT_SYMBOL_GPL(__pm_runtime_set_status);

static void __pm_runtime_barrier(struct device *dev)
{
	pm_runtime_deactivate_timer(dev);

	if (dev->power.request_pending) {
		dev->power.request = RPM_REQ_NONE;
		spin_unlock_irq(&dev->power.lock);

		cancel_work_sync(&dev->power.work);

		spin_lock_irq(&dev->power.lock);
		dev->power.request_pending = false;
	}

	if (dev->power.runtime_status == RPM_SUSPENDING
	    || dev->power.runtime_status == RPM_RESUMING
	    || dev->power.idle_notification) {
		DEFINE_WAIT(wait);

		/* Suspend, wake-up or idle notification in progress. */
		for (;;) {
			prepare_to_wait(&dev->power.wait_queue, &wait,
					TASK_UNINTERRUPTIBLE);
			if (dev->power.runtime_status != RPM_SUSPENDING
			    && dev->power.runtime_status != RPM_RESUMING
			    && !dev->power.idle_notification)
				break;
			spin_unlock_irq(&dev->power.lock);

			schedule();

			spin_lock_irq(&dev->power.lock);
		}
		finish_wait(&dev->power.wait_queue, &wait);
	}
}

int pm_runtime_barrier(struct device *dev)
{
	int retval = 0;

	pm_runtime_get_noresume(dev);
	spin_lock_irq(&dev->power.lock);

	if (dev->power.request_pending
	    && dev->power.request == RPM_REQ_RESUME) {
		__pm_runtime_resume(dev, false);
		retval = 1;
	}

	__pm_runtime_barrier(dev);

	spin_unlock_irq(&dev->power.lock);
	pm_runtime_put_noidle(dev);

	return retval;
}
EXPORT_SYMBOL_GPL(pm_runtime_barrier);

void __pm_runtime_disable(struct device *dev, bool check_resume)
{
	spin_lock_irq(&dev->power.lock);

	if (dev->power.disable_depth > 0) {
		dev->power.disable_depth++;
		goto out;
	}

	/*
	 * Wake up the device if there's a resume request pending, because that
	 * means there probably is some I/O to process and disabling run-time PM
	 * shouldn't prevent the device from processing the I/O.
	 */
	if (check_resume && dev->power.request_pending
	    && dev->power.request == RPM_REQ_RESUME) {
		/*
		 * Prevent suspends and idle notifications from being carried
		 * out after we have woken up the device.
		 */
		pm_runtime_get_noresume(dev);

		__pm_runtime_resume(dev, false);

		pm_runtime_put_noidle(dev);
	}

	if (!dev->power.disable_depth++)
		__pm_runtime_barrier(dev);

 out:
	spin_unlock_irq(&dev->power.lock);
}
EXPORT_SYMBOL_GPL(__pm_runtime_disable);

void pm_runtime_enable(struct device *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->power.lock, flags);

	if (dev->power.disable_depth > 0)
		dev->power.disable_depth--;
	else
		dev_warn(dev, "Unbalanced %s!\n", __func__);

	spin_unlock_irqrestore(&dev->power.lock, flags);
}
EXPORT_SYMBOL_GPL(pm_runtime_enable);

void pm_runtime_forbid(struct device *dev)
{
	spin_lock_irq(&dev->power.lock);
	if (!dev->power.runtime_auto)
		goto out;

	dev->power.runtime_auto = false;
	atomic_inc(&dev->power.usage_count);
	__pm_runtime_resume(dev, false);

 out:
	spin_unlock_irq(&dev->power.lock);
}
EXPORT_SYMBOL_GPL(pm_runtime_forbid);

void pm_runtime_allow(struct device *dev)
{
	spin_lock_irq(&dev->power.lock);
	if (dev->power.runtime_auto)
		goto out;

	dev->power.runtime_auto = true;
	if (atomic_dec_and_test(&dev->power.usage_count))
		__pm_runtime_idle(dev);

 out:
	spin_unlock_irq(&dev->power.lock);
}
EXPORT_SYMBOL_GPL(pm_runtime_allow);

void pm_runtime_init(struct device *dev)
{
	spin_lock_init(&dev->power.lock);

	dev->power.runtime_status = RPM_SUSPENDED;
	dev->power.idle_notification = false;

	dev->power.disable_depth = 1;
	atomic_set(&dev->power.usage_count, 0);

	dev->power.runtime_error = 0;

	atomic_set(&dev->power.child_count, 0);
	pm_suspend_ignore_children(dev, false);
	dev->power.runtime_auto = true;

	dev->power.request_pending = false;
	dev->power.request = RPM_REQ_NONE;
	dev->power.deferred_resume = false;
	INIT_WORK(&dev->power.work, pm_runtime_work);

	dev->power.timer_expires = 0;
	setup_timer(&dev->power.suspend_timer, pm_suspend_timer_fn,
			(unsigned long)dev);

	init_waitqueue_head(&dev->power.wait_queue);
}

void pm_runtime_remove(struct device *dev)
{
	__pm_runtime_disable(dev, false);

	/* Change the status back to 'suspended' to match the initial status. */
	if (dev->power.runtime_status == RPM_ACTIVE)
		pm_runtime_set_suspended(dev);
}
