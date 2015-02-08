

#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/freezer.h>

static inline void frozen_process(void)
{
	if (!unlikely(current->flags & PF_NOFREEZE)) {
		current->flags |= PF_FROZEN;
		wmb();
	}
	clear_freeze_flag(current);
}

/* Refrigerator is place where frozen processes are stored :-). */
void refrigerator(void)
{
	/* Hmm, should we be allowed to suspend when there are realtime
	   processes around? */
	long save;

	task_lock(current);
	if (freezing(current)) {
		frozen_process();
		task_unlock(current);
	} else {
		task_unlock(current);
		return;
	}
	save = current->state;
	pr_debug("%s entered refrigerator\n", current->comm);

	spin_lock_irq(&current->sighand->siglock);
	recalc_sigpending(); /* We sent fake signal, clean it up */
	spin_unlock_irq(&current->sighand->siglock);

	/* prevent accounting of that task to load */
	current->flags |= PF_FREEZING;

	for (;;) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		if (!frozen(current))
			break;
		schedule();
	}

	/* Remove the accounting blocker */
	current->flags &= ~PF_FREEZING;

	pr_debug("%s left refrigerator\n", current->comm);
	__set_current_state(save);
}
EXPORT_SYMBOL(refrigerator);

static void fake_signal_wake_up(struct task_struct *p)
{
	unsigned long flags;

	spin_lock_irqsave(&p->sighand->siglock, flags);
	signal_wake_up(p, 0);
	spin_unlock_irqrestore(&p->sighand->siglock, flags);
}

bool freeze_task(struct task_struct *p, bool sig_only)
{
	/*
	 * We first check if the task is freezing and next if it has already
	 * been frozen to avoid the race with frozen_process() which first marks
	 * the task as frozen and next clears its TIF_FREEZE.
	 */
	if (!freezing(p)) {
		rmb();
		if (frozen(p))
			return false;

		if (!sig_only || should_send_signal(p))
			set_freeze_flag(p);
		else
			return false;
	}

	if (should_send_signal(p)) {
		if (!signal_pending(p))
			fake_signal_wake_up(p);
	} else if (sig_only) {
		return false;
	} else {
		wake_up_state(p, TASK_INTERRUPTIBLE);
	}

	return true;
}

void cancel_freezing(struct task_struct *p)
{
	unsigned long flags;

	if (freezing(p)) {
		pr_debug("  clean up: %s\n", p->comm);
		clear_freeze_flag(p);
		spin_lock_irqsave(&p->sighand->siglock, flags);
		recalc_sigpending_and_wake(p);
		spin_unlock_irqrestore(&p->sighand->siglock, flags);
	}
}

static int __thaw_process(struct task_struct *p)
{
	if (frozen(p)) {
		p->flags &= ~PF_FROZEN;
		return 1;
	}
	clear_freeze_flag(p);
	return 0;
}

int thaw_process(struct task_struct *p)
{
	task_lock(p);
	if (__thaw_process(p) == 1) {
		task_unlock(p);
		wake_up_process(p);
		return 1;
	}
	task_unlock(p);
	return 0;
}
EXPORT_SYMBOL(thaw_process);
