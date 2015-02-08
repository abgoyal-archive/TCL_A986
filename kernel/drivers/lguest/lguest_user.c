
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/eventfd.h>
#include <linux/file.h>
#include <linux/slab.h>
#include "lg.h"

bool send_notify_to_eventfd(struct lg_cpu *cpu)
{
	unsigned int i;
	struct lg_eventfd_map *map;

	/*
	 * This "rcu_read_lock()" helps track when someone is still looking at
	 * the (RCU-using) eventfds array.  It's not actually a lock at all;
	 * indeed it's a noop in many configurations.  (You didn't expect me to
	 * explain all the RCU secrets here, did you?)
	 */
	rcu_read_lock();
	/*
	 * rcu_dereference is the counter-side of rcu_assign_pointer(); it
	 * makes sure we don't access the memory pointed to by
	 * cpu->lg->eventfds before cpu->lg->eventfds is set.  Sounds crazy,
	 * but Alpha allows this!  Paul McKenney points out that a really
	 * aggressive compiler could have the same effect:
	 *   http://lists.ozlabs.org/pipermail/lguest/2009-July/001560.html
	 *
	 * So play safe, use rcu_dereference to get the rcu-protected pointer:
	 */
	map = rcu_dereference(cpu->lg->eventfds);
	/*
	 * Simple array search: even if they add an eventfd while we do this,
	 * we'll continue to use the old array and just won't see the new one.
	 */
	for (i = 0; i < map->num; i++) {
		if (map->map[i].addr == cpu->pending_notify) {
			eventfd_signal(map->map[i].event, 1);
			cpu->pending_notify = 0;
			break;
		}
	}
	/* We're done with the rcu-protected variable cpu->lg->eventfds. */
	rcu_read_unlock();

	/* If we cleared the notification, it's because we found a match. */
	return cpu->pending_notify == 0;
}

static int add_eventfd(struct lguest *lg, unsigned long addr, int fd)
{
	struct lg_eventfd_map *new, *old = lg->eventfds;

	/*
	 * We don't allow notifications on value 0 anyway (pending_notify of
	 * 0 means "nothing pending").
	 */
	if (!addr)
		return -EINVAL;

	/*
	 * Replace the old array with the new one, carefully: others can
	 * be accessing it at the same time.
	 */
	new = kmalloc(sizeof(*new) + sizeof(new->map[0]) * (old->num + 1),
		      GFP_KERNEL);
	if (!new)
		return -ENOMEM;

	/* First make identical copy. */
	memcpy(new->map, old->map, sizeof(old->map[0]) * old->num);
	new->num = old->num;

	/* Now append new entry. */
	new->map[new->num].addr = addr;
	new->map[new->num].event = eventfd_ctx_fdget(fd);
	if (IS_ERR(new->map[new->num].event)) {
		int err =  PTR_ERR(new->map[new->num].event);
		kfree(new);
		return err;
	}
	new->num++;

	/*
	 * Now put new one in place: rcu_assign_pointer() is a fancy way of
	 * doing "lg->eventfds = new", but it uses memory barriers to make
	 * absolutely sure that the contents of "new" written above is nailed
	 * down before we actually do the assignment.
	 *
	 * We have to think about these kinds of things when we're operating on
	 * live data without locks.
	 */
	rcu_assign_pointer(lg->eventfds, new);

	/*
	 * We're not in a big hurry.  Wait until noone's looking at old
	 * version, then free it.
	 */
	synchronize_rcu();
	kfree(old);

	return 0;
}

static int attach_eventfd(struct lguest *lg, const unsigned long __user *input)
{
	unsigned long addr, fd;
	int err;

	if (get_user(addr, input) != 0)
		return -EFAULT;
	input++;
	if (get_user(fd, input) != 0)
		return -EFAULT;

	/*
	 * Just make sure two callers don't add eventfds at once.  We really
	 * only need to lock against callers adding to the same Guest, so using
	 * the Big Lguest Lock is overkill.  But this is setup, not a fast path.
	 */
	mutex_lock(&lguest_lock);
	err = add_eventfd(lg, addr, fd);
	mutex_unlock(&lguest_lock);

	return err;
}

static int user_send_irq(struct lg_cpu *cpu, const unsigned long __user *input)
{
	unsigned long irq;

	if (get_user(irq, input) != 0)
		return -EFAULT;
	if (irq >= LGUEST_IRQS)
		return -EINVAL;

	/*
	 * Next time the Guest runs, the core code will see if it can deliver
	 * this interrupt.
	 */
	set_interrupt(cpu, irq);
	return 0;
}

static ssize_t read(struct file *file, char __user *user, size_t size,loff_t*o)
{
	struct lguest *lg = file->private_data;
	struct lg_cpu *cpu;
	unsigned int cpu_id = *o;

	/* You must write LHREQ_INITIALIZE first! */
	if (!lg)
		return -EINVAL;

	/* Watch out for arbitrary vcpu indexes! */
	if (cpu_id >= lg->nr_cpus)
		return -EINVAL;

	cpu = &lg->cpus[cpu_id];

	/* If you're not the task which owns the Guest, go away. */
	if (current != cpu->tsk)
		return -EPERM;

	/* If the Guest is already dead, we indicate why */
	if (lg->dead) {
		size_t len;

		/* lg->dead either contains an error code, or a string. */
		if (IS_ERR(lg->dead))
			return PTR_ERR(lg->dead);

		/* We can only return as much as the buffer they read with. */
		len = min(size, strlen(lg->dead)+1);
		if (copy_to_user(user, lg->dead, len) != 0)
			return -EFAULT;
		return len;
	}

	/*
	 * If we returned from read() last time because the Guest sent I/O,
	 * clear the flag.
	 */
	if (cpu->pending_notify)
		cpu->pending_notify = 0;

	/* Run the Guest until something interesting happens. */
	return run_guest(cpu, (unsigned long __user *)user);
}

static int lg_cpu_start(struct lg_cpu *cpu, unsigned id, unsigned long start_ip)
{
	/* We have a limited number the number of CPUs in the lguest struct. */
	if (id >= ARRAY_SIZE(cpu->lg->cpus))
		return -EINVAL;

	/* Set up this CPU's id, and pointer back to the lguest struct. */
	cpu->id = id;
	cpu->lg = container_of((cpu - id), struct lguest, cpus[0]);
	cpu->lg->nr_cpus++;

	/* Each CPU has a timer it can set. */
	init_clockdev(cpu);

	/*
	 * We need a complete page for the Guest registers: they are accessible
	 * to the Guest and we can only grant it access to whole pages.
	 */
	cpu->regs_page = get_zeroed_page(GFP_KERNEL);
	if (!cpu->regs_page)
		return -ENOMEM;

	/* We actually put the registers at the bottom of the page. */
	cpu->regs = (void *)cpu->regs_page + PAGE_SIZE - sizeof(*cpu->regs);

	/*
	 * Now we initialize the Guest's registers, handing it the start
	 * address.
	 */
	lguest_arch_setup_regs(cpu, start_ip);

	/*
	 * We keep a pointer to the Launcher task (ie. current task) for when
	 * other Guests want to wake this one (eg. console input).
	 */
	cpu->tsk = current;

	/*
	 * We need to keep a pointer to the Launcher's memory map, because if
	 * the Launcher dies we need to clean it up.  If we don't keep a
	 * reference, it is destroyed before close() is called.
	 */
	cpu->mm = get_task_mm(cpu->tsk);

	/*
	 * We remember which CPU's pages this Guest used last, for optimization
	 * when the same Guest runs on the same CPU twice.
	 */
	cpu->last_pages = NULL;

	/* No error == success. */
	return 0;
}

static int initialize(struct file *file, const unsigned long __user *input)
{
	/* "struct lguest" contains all we (the Host) know about a Guest. */
	struct lguest *lg;
	int err;
	unsigned long args[3];

	/*
	 * We grab the Big Lguest lock, which protects against multiple
	 * simultaneous initializations.
	 */
	mutex_lock(&lguest_lock);
	/* You can't initialize twice!  Close the device and start again... */
	if (file->private_data) {
		err = -EBUSY;
		goto unlock;
	}

	if (copy_from_user(args, input, sizeof(args)) != 0) {
		err = -EFAULT;
		goto unlock;
	}

	lg = kzalloc(sizeof(*lg), GFP_KERNEL);
	if (!lg) {
		err = -ENOMEM;
		goto unlock;
	}

	lg->eventfds = kmalloc(sizeof(*lg->eventfds), GFP_KERNEL);
	if (!lg->eventfds) {
		err = -ENOMEM;
		goto free_lg;
	}
	lg->eventfds->num = 0;

	/* Populate the easy fields of our "struct lguest" */
	lg->mem_base = (void __user *)args[0];
	lg->pfn_limit = args[1];

	/* This is the first cpu (cpu 0) and it will start booting at args[2] */
	err = lg_cpu_start(&lg->cpus[0], 0, args[2]);
	if (err)
		goto free_eventfds;

	/*
	 * Initialize the Guest's shadow page tables, using the toplevel
	 * address the Launcher gave us.  This allocates memory, so can fail.
	 */
	err = init_guest_pagetable(lg);
	if (err)
		goto free_regs;

	/* We keep our "struct lguest" in the file's private_data. */
	file->private_data = lg;

	mutex_unlock(&lguest_lock);

	/* And because this is a write() call, we return the length used. */
	return sizeof(args);

free_regs:
	/* FIXME: This should be in free_vcpu */
	free_page(lg->cpus[0].regs_page);
free_eventfds:
	kfree(lg->eventfds);
free_lg:
	kfree(lg);
unlock:
	mutex_unlock(&lguest_lock);
	return err;
}

static ssize_t write(struct file *file, const char __user *in,
		     size_t size, loff_t *off)
{
	/*
	 * Once the Guest is initialized, we hold the "struct lguest" in the
	 * file private data.
	 */
	struct lguest *lg = file->private_data;
	const unsigned long __user *input = (const unsigned long __user *)in;
	unsigned long req;
	struct lg_cpu *uninitialized_var(cpu);
	unsigned int cpu_id = *off;

	/* The first value tells us what this request is. */
	if (get_user(req, input) != 0)
		return -EFAULT;
	input++;

	/* If you haven't initialized, you must do that first. */
	if (req != LHREQ_INITIALIZE) {
		if (!lg || (cpu_id >= lg->nr_cpus))
			return -EINVAL;
		cpu = &lg->cpus[cpu_id];

		/* Once the Guest is dead, you can only read() why it died. */
		if (lg->dead)
			return -ENOENT;
	}

	switch (req) {
	case LHREQ_INITIALIZE:
		return initialize(file, input);
	case LHREQ_IRQ:
		return user_send_irq(cpu, input);
	case LHREQ_EVENTFD:
		return attach_eventfd(lg, input);
	default:
		return -EINVAL;
	}
}

static int close(struct inode *inode, struct file *file)
{
	struct lguest *lg = file->private_data;
	unsigned int i;

	/* If we never successfully initialized, there's nothing to clean up */
	if (!lg)
		return 0;

	/*
	 * We need the big lock, to protect from inter-guest I/O and other
	 * Launchers initializing guests.
	 */
	mutex_lock(&lguest_lock);

	/* Free up the shadow page tables for the Guest. */
	free_guest_pagetable(lg);

	for (i = 0; i < lg->nr_cpus; i++) {
		/* Cancels the hrtimer set via LHCALL_SET_CLOCKEVENT. */
		hrtimer_cancel(&lg->cpus[i].hrt);
		/* We can free up the register page we allocated. */
		free_page(lg->cpus[i].regs_page);
		/*
		 * Now all the memory cleanups are done, it's safe to release
		 * the Launcher's memory management structure.
		 */
		mmput(lg->cpus[i].mm);
	}

	/* Release any eventfds they registered. */
	for (i = 0; i < lg->eventfds->num; i++)
		eventfd_ctx_put(lg->eventfds->map[i].event);
	kfree(lg->eventfds);

	/*
	 * If lg->dead doesn't contain an error code it will be NULL or a
	 * kmalloc()ed string, either of which is ok to hand to kfree().
	 */
	if (!IS_ERR(lg->dead))
		kfree(lg->dead);
	/* Free the memory allocated to the lguest_struct */
	kfree(lg);
	/* Release lock and exit. */
	mutex_unlock(&lguest_lock);

	return 0;
}

static const struct file_operations lguest_fops = {
	.owner	 = THIS_MODULE,
	.release = close,
	.write	 = write,
	.read	 = read,
};

static struct miscdevice lguest_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "lguest",
	.fops	= &lguest_fops,
};

int __init lguest_device_init(void)
{
	return misc_register(&lguest_dev);
}

void __exit lguest_device_remove(void)
{
	misc_deregister(&lguest_dev);
}
