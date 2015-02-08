

#include <linux/module.h>

#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/fcntl.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/major.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/smp_lock.h>
#include <linux/sysctl.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/bitops.h>
#include <linux/devpts_fs.h>
#include <linux/slab.h>

#include <asm/system.h>

#ifdef CONFIG_UNIX98_PTYS
static struct tty_driver *ptm_driver;
static struct tty_driver *pts_driver;
#endif

static void pty_close(struct tty_struct *tty, struct file *filp)
{
	BUG_ON(!tty);
	if (tty->driver->subtype == PTY_TYPE_MASTER)
		WARN_ON(tty->count > 1);
	else {
		if (tty->count > 2)
			return;
	}
	wake_up_interruptible(&tty->read_wait);
	wake_up_interruptible(&tty->write_wait);
	tty->packet = 0;
        tty->peer_stops = 0;    
	if (!tty->link)
		return;
	tty->link->packet = 0;
        tty->link->peer_stops = 0;
	set_bit(TTY_OTHER_CLOSED, &tty->link->flags);
	wake_up_interruptible(&tty->link->read_wait);
	wake_up_interruptible(&tty->link->write_wait);
	if (tty->driver->subtype == PTY_TYPE_MASTER) {
		set_bit(TTY_OTHER_CLOSED, &tty->flags);
#ifdef CONFIG_UNIX98_PTYS
		if (tty->driver == ptm_driver)
			devpts_pty_kill(tty->link);
#endif
		tty_vhangup(tty->link);
	}
}

static void pty_unthrottle(struct tty_struct *tty)
{
	tty_wakeup(tty->link);
	set_bit(TTY_THROTTLED, &tty->flags);
}


static int pty_space(struct tty_struct *to)
{
	int n = 8192 - to->buf.memory_used;
	if (n < 0)
		return 0;
	return n;
}


static int pty_write(struct tty_struct *tty, const unsigned char *buf, int c)
{
	struct tty_struct *to = tty->link;

	if (tty->stopped || tty->peer_stops) {
		return 0;
	}

	if (c > 0) {
		/* Stuff the data into the input queue of the other end */
		c = tty_insert_flip_string(to, buf, c);
		/* And shovel */
		if (c) {
			tty_flip_buffer_push(to);
			tty_wakeup(tty);
		}
	}
	return c;
}


static int pty_write_room(struct tty_struct *tty)
{
	if (tty->stopped || tty->peer_stops) {
		return 0;
	}

	return pty_space(tty->link);
}


static int pty_chars_in_buffer(struct tty_struct *tty)
{
	return 0;
}

/* Set the lock flag on a pty */
static int pty_set_lock(struct tty_struct *tty, int __user *arg)
{
	int val;
	if (get_user(val, arg))
		return -EFAULT;
	if (val)
		set_bit(TTY_PTY_LOCK, &tty->flags);
	else
		clear_bit(TTY_PTY_LOCK, &tty->flags);
	return 0;
}

static void pty_flush_buffer(struct tty_struct *tty)
{
	struct tty_struct *to = tty->link;
	unsigned long flags;

	if (!to)
		return;
	/* tty_buffer_flush(to); FIXME */
	if (to->packet) {
		spin_lock_irqsave(&tty->ctrl_lock, flags);
		tty->ctrl_status |= TIOCPKT_FLUSHWRITE;
		wake_up_interruptible(&to->read_wait);
		spin_unlock_irqrestore(&tty->ctrl_lock, flags);
	}
}

static int pty_open(struct tty_struct *tty, struct file *filp)
{
	int	retval = -ENODEV;

	if (!tty || !tty->link)
		goto out;

	retval = -EIO;
	if (test_bit(TTY_OTHER_CLOSED, &tty->flags))
		goto out;
	if (test_bit(TTY_PTY_LOCK, &tty->link->flags))
		goto out;
	if (tty->link->count != 1)
		goto out;

	clear_bit(TTY_OTHER_CLOSED, &tty->link->flags);
	set_bit(TTY_THROTTLED, &tty->flags);
	retval = 0;
out:
	return retval;
}

static void pty_set_termios(struct tty_struct *tty,
					struct ktermios *old_termios)
{
	tty->termios->c_cflag &= ~(CSIZE | PARENB);
	tty->termios->c_cflag |= (CS8 | CREAD);
}


int pty_resize(struct tty_struct *tty,  struct winsize *ws)
{
	struct pid *pgrp, *rpgrp;
	unsigned long flags;
	struct tty_struct *pty = tty->link;

	/* For a PTY we need to lock the tty side */
	mutex_lock(&tty->termios_mutex);
	if (!memcmp(ws, &tty->winsize, sizeof(*ws)))
		goto done;

	/* Get the PID values and reference them so we can
	   avoid holding the tty ctrl lock while sending signals.
	   We need to lock these individually however. */

	spin_lock_irqsave(&tty->ctrl_lock, flags);
	pgrp = get_pid(tty->pgrp);
	spin_unlock_irqrestore(&tty->ctrl_lock, flags);

	spin_lock_irqsave(&pty->ctrl_lock, flags);
	rpgrp = get_pid(pty->pgrp);
	spin_unlock_irqrestore(&pty->ctrl_lock, flags);

	if (pgrp)
		kill_pgrp(pgrp, SIGWINCH, 1);
	if (rpgrp != pgrp && rpgrp)
		kill_pgrp(rpgrp, SIGWINCH, 1);

	put_pid(pgrp);
	put_pid(rpgrp);

	tty->winsize = *ws;
	pty->winsize = *ws;	/* Never used so will go away soon */
done:
	mutex_unlock(&tty->termios_mutex);
	return 0;
}

/* Traditional BSD devices */
#ifdef CONFIG_LEGACY_PTYS

static int pty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct tty_struct *o_tty;
	int idx = tty->index;
	int retval;

	o_tty = alloc_tty_struct();
	if (!o_tty)
		return -ENOMEM;
	if (!try_module_get(driver->other->owner)) {
		/* This cannot in fact currently happen */
		free_tty_struct(o_tty);
		return -ENOMEM;
	}
	initialize_tty_struct(o_tty, driver->other, idx);

	/* We always use new tty termios data so we can do this
	   the easy way .. */
	retval = tty_init_termios(tty);
	if (retval)
		goto free_mem_out;

	retval = tty_init_termios(o_tty);
	if (retval) {
		tty_free_termios(tty);
		goto free_mem_out;
	}

	/*
	 * Everything allocated ... set up the o_tty structure.
	 */
	driver->other->ttys[idx] = o_tty;
	tty_driver_kref_get(driver->other);
	if (driver->subtype == PTY_TYPE_MASTER)
		o_tty->count++;
	/* Establish the links in both directions */
	tty->link   = o_tty;
	o_tty->link = tty;

	tty_driver_kref_get(driver);
	tty->count++;
	driver->ttys[idx] = tty;
	return 0;
free_mem_out:
	module_put(o_tty->driver->owner);
	free_tty_struct(o_tty);
	return -ENOMEM;
}

static int pty_bsd_ioctl(struct tty_struct *tty, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case TIOCSPTLCK: /* Set PT Lock (disallow slave open) */
		return pty_set_lock(tty, (int __user *) arg);
	}
	return -ENOIOCTLCMD;
}

static int legacy_count = CONFIG_LEGACY_PTY_COUNT;
module_param(legacy_count, int, 0);

static const struct tty_operations master_pty_ops_bsd = {
	.install = pty_install,
	.open = pty_open,
	.close = pty_close,
	.write = pty_write,
	.write_room = pty_write_room,
	.flush_buffer = pty_flush_buffer,
	.chars_in_buffer = pty_chars_in_buffer,
	.unthrottle = pty_unthrottle,
	.set_termios = pty_set_termios,
	.ioctl = pty_bsd_ioctl,
	.resize = pty_resize
};

static const struct tty_operations slave_pty_ops_bsd = {
	.install = pty_install,
	.open = pty_open,
	.close = pty_close,
	.write = pty_write,
	.write_room = pty_write_room,
	.flush_buffer = pty_flush_buffer,
	.chars_in_buffer = pty_chars_in_buffer,
	.unthrottle = pty_unthrottle,
	.set_termios = pty_set_termios,
	.resize = pty_resize
};

static void __init legacy_pty_init(void)
{
	struct tty_driver *pty_driver, *pty_slave_driver;

	if (legacy_count <= 0)
		return;

	pty_driver = alloc_tty_driver(legacy_count);
	if (!pty_driver)
		panic("Couldn't allocate pty driver");

	pty_slave_driver = alloc_tty_driver(legacy_count);
	if (!pty_slave_driver)
		panic("Couldn't allocate pty slave driver");

	pty_driver->owner = THIS_MODULE;
	pty_driver->driver_name = "pty_master";
	pty_driver->name = "pty";
	pty_driver->major = PTY_MASTER_MAJOR;
	pty_driver->minor_start = 0;
	pty_driver->type = TTY_DRIVER_TYPE_PTY;
	pty_driver->subtype = PTY_TYPE_MASTER;
	pty_driver->init_termios = tty_std_termios;
	pty_driver->init_termios.c_iflag = 0;
	pty_driver->init_termios.c_oflag = 0;
	pty_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	pty_driver->init_termios.c_lflag = 0;
	pty_driver->init_termios.c_ispeed = 38400;
	pty_driver->init_termios.c_ospeed = 38400;
	pty_driver->flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW;
	pty_driver->other = pty_slave_driver;
	tty_set_operations(pty_driver, &master_pty_ops_bsd);

	pty_slave_driver->owner = THIS_MODULE;
	pty_slave_driver->driver_name = "pty_slave";
	pty_slave_driver->name = "ttyp";
	pty_slave_driver->major = PTY_SLAVE_MAJOR;
	pty_slave_driver->minor_start = 0;
	pty_slave_driver->type = TTY_DRIVER_TYPE_PTY;
	pty_slave_driver->subtype = PTY_TYPE_SLAVE;
	pty_slave_driver->init_termios = tty_std_termios;
	pty_slave_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	pty_slave_driver->init_termios.c_ispeed = 38400;
	pty_slave_driver->init_termios.c_ospeed = 38400;
	pty_slave_driver->flags = TTY_DRIVER_RESET_TERMIOS |
					TTY_DRIVER_REAL_RAW;
	pty_slave_driver->other = pty_driver;
	tty_set_operations(pty_slave_driver, &slave_pty_ops_bsd);

	if (tty_register_driver(pty_driver))
		panic("Couldn't register pty driver");
	if (tty_register_driver(pty_slave_driver))
		panic("Couldn't register pty slave driver");
}
#else
static inline void legacy_pty_init(void) { }
#endif

/* Unix98 devices */
#ifdef CONFIG_UNIX98_PTYS
int pty_limit = NR_UNIX98_PTY_DEFAULT;
static int pty_limit_min;
static int pty_limit_max = NR_UNIX98_PTY_MAX;
static int pty_count;

static struct cdev ptmx_cdev;

static struct ctl_table pty_table[] = {
	{
		.procname	= "max",
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.data		= &pty_limit,
		.proc_handler	= proc_dointvec_minmax,
		.extra1		= &pty_limit_min,
		.extra2		= &pty_limit_max,
	}, {
		.procname	= "nr",
		.maxlen		= sizeof(int),
		.mode		= 0444,
		.data		= &pty_count,
		.proc_handler	= proc_dointvec,
	}, 
	{}
};

static struct ctl_table pty_kern_table[] = {
	{
		.procname	= "pty",
		.mode		= 0555,
		.child		= pty_table,
	},
	{}
};

static struct ctl_table pty_root_table[] = {
	{
		.procname	= "kernel",
		.mode		= 0555,
		.child		= pty_kern_table,
	},
	{}
};

#define TCOOFF 0
#define TCOON 1
#define TCIOFF 2
#define TCION 3

static int pty_unix98_ioctl(struct tty_struct *tty, struct file *file,
			    unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case TIOCSPTLCK: /* Set PT Lock (disallow slave open) */
		return pty_set_lock(tty, (int __user *)arg);
	case TIOCGPTN: /* Get PT Number */
		return put_user(tty->index, (unsigned int __user *)arg);
	case TCXONC: /* Flow Control */
		switch (arg) {
                case TCIOFF:
                     tty->link->peer_stops=1;
                     break;
                case TCION:
                     tty->link->peer_stops=0;           
                     if (waitqueue_active(&tty->link->write_wait))
                         wake_up_interruptible(&tty->link->write_wait);
                     break;
                default:
                     return -EINVAL;
		}
                return 0;
	}

	return -ENOIOCTLCMD;
}


static struct tty_struct *ptm_unix98_lookup(struct tty_driver *driver,
		struct inode *ptm_inode, int idx)
{
	struct tty_struct *tty = devpts_get_tty(ptm_inode, idx);
	if (tty)
		tty = tty->link;
	return tty;
}


static struct tty_struct *pts_unix98_lookup(struct tty_driver *driver,
		struct inode *pts_inode, int idx)
{
	struct tty_struct *tty = devpts_get_tty(pts_inode, idx);
	/* Master must be open before slave */
	if (!tty)
		return ERR_PTR(-EIO);
	return tty;
}

static void pty_unix98_shutdown(struct tty_struct *tty)
{
	/* We have our own method as we don't use the tty index */
	kfree(tty->termios);
}


static int pty_unix98_install(struct tty_driver *driver, struct tty_struct *tty)
{
	struct tty_struct *o_tty;
	int idx = tty->index;

	o_tty = alloc_tty_struct();
	if (!o_tty)
		return -ENOMEM;
	if (!try_module_get(driver->other->owner)) {
		/* This cannot in fact currently happen */
		free_tty_struct(o_tty);
		return -ENOMEM;
	}
	initialize_tty_struct(o_tty, driver->other, idx);

	tty->termios = kzalloc(sizeof(struct ktermios[2]), GFP_KERNEL);
	if (tty->termios == NULL)
		goto free_mem_out;
	*tty->termios = driver->init_termios;
	tty->termios_locked = tty->termios + 1;

	o_tty->termios = kzalloc(sizeof(struct ktermios[2]), GFP_KERNEL);
	if (o_tty->termios == NULL)
		goto free_mem_out;
	*o_tty->termios = driver->other->init_termios;
	o_tty->termios_locked = o_tty->termios + 1;

	tty_driver_kref_get(driver->other);
	if (driver->subtype == PTY_TYPE_MASTER)
		o_tty->count++;
	/* Establish the links in both directions */
	tty->link   = o_tty;
	o_tty->link = tty;
	/*
	 * All structures have been allocated, so now we install them.
	 * Failures after this point use release_tty to clean up, so
	 * there's no need to null out the local pointers.
	 */
	tty_driver_kref_get(driver);
	tty->count++;
	pty_count++;
	return 0;
free_mem_out:
	kfree(o_tty->termios);
	module_put(o_tty->driver->owner);
	free_tty_struct(o_tty);
	kfree(tty->termios);
	return -ENOMEM;
}

static void pty_unix98_remove(struct tty_driver *driver, struct tty_struct *tty)
{
	pty_count--;
}

static const struct tty_operations ptm_unix98_ops = {
	.lookup = ptm_unix98_lookup,
	.install = pty_unix98_install,
	.remove = pty_unix98_remove,
	.open = pty_open,
	.close = pty_close,
	.write = pty_write,
	.write_room = pty_write_room,
	.flush_buffer = pty_flush_buffer,
	.chars_in_buffer = pty_chars_in_buffer,
	.unthrottle = pty_unthrottle,
	.set_termios = pty_set_termios,
	.ioctl = pty_unix98_ioctl,
	.shutdown = pty_unix98_shutdown,
	.resize = pty_resize
};

static const struct tty_operations pty_unix98_ops = {
	.lookup = pts_unix98_lookup,
	.install = pty_unix98_install,
	.remove = pty_unix98_remove,
	.open = pty_open,
	.close = pty_close,
	.write = pty_write,
	.write_room = pty_write_room,
	.flush_buffer = pty_flush_buffer,
	.chars_in_buffer = pty_chars_in_buffer,
	.unthrottle = pty_unthrottle,
	.set_termios = pty_set_termios,
	.ioctl = pty_unix98_ioctl,
	.shutdown = pty_unix98_shutdown
};


static int __ptmx_open(struct inode *inode, struct file *filp)
{
	struct tty_struct *tty;
	int retval;
	int index;

	nonseekable_open(inode, filp);

	/* find a device that is not in use. */
	index = devpts_new_index(inode);
	if (index < 0)
		return index;

	mutex_lock(&tty_mutex);
	tty = tty_init_dev(ptm_driver, index, 1);
	mutex_unlock(&tty_mutex);

	if (IS_ERR(tty)) {
		retval = PTR_ERR(tty);
		goto out;
	}

	set_bit(TTY_PTY_LOCK, &tty->flags); /* LOCK THE SLAVE */
	filp->private_data = tty;
	file_move(filp, &tty->tty_files);

	retval = devpts_pty_new(inode, tty->link);
	if (retval)
		goto out1;

	retval = ptm_driver->ops->open(tty, filp);
	if (!retval)
		return 0;
out1:
	tty_release(inode, filp);
	return retval;
out:
	devpts_kill_index(inode, index);
	return retval;
}

static int ptmx_open(struct inode *inode, struct file *filp)
{
	int ret;

	lock_kernel();
	ret = __ptmx_open(inode, filp);
	unlock_kernel();
	return ret;
}

static struct file_operations ptmx_fops;

static void __init unix98_pty_init(void)
{
	ptm_driver = alloc_tty_driver(NR_UNIX98_PTY_MAX);
	if (!ptm_driver)
		panic("Couldn't allocate Unix98 ptm driver");
	pts_driver = alloc_tty_driver(NR_UNIX98_PTY_MAX);
	if (!pts_driver)
		panic("Couldn't allocate Unix98 pts driver");

	ptm_driver->owner = THIS_MODULE;
	ptm_driver->driver_name = "pty_master";
	ptm_driver->name = "ptm";
	ptm_driver->major = UNIX98_PTY_MASTER_MAJOR;
	ptm_driver->minor_start = 0;
	ptm_driver->type = TTY_DRIVER_TYPE_PTY;
	ptm_driver->subtype = PTY_TYPE_MASTER;
	ptm_driver->init_termios = tty_std_termios;
	ptm_driver->init_termios.c_iflag = 0;
	ptm_driver->init_termios.c_oflag = 0;
	ptm_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	ptm_driver->init_termios.c_lflag = 0;
	ptm_driver->init_termios.c_ispeed = 38400;
	ptm_driver->init_termios.c_ospeed = 38400;
	ptm_driver->flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW |
		TTY_DRIVER_DYNAMIC_DEV | TTY_DRIVER_DEVPTS_MEM;
	ptm_driver->other = pts_driver;
	tty_set_operations(ptm_driver, &ptm_unix98_ops);

	pts_driver->owner = THIS_MODULE;
	pts_driver->driver_name = "pty_slave";
	pts_driver->name = "pts";
	pts_driver->major = UNIX98_PTY_SLAVE_MAJOR;
	pts_driver->minor_start = 0;
	pts_driver->type = TTY_DRIVER_TYPE_PTY;
	pts_driver->subtype = PTY_TYPE_SLAVE;
	pts_driver->init_termios = tty_std_termios;
	pts_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	pts_driver->init_termios.c_ispeed = 38400;
	pts_driver->init_termios.c_ospeed = 38400;
	pts_driver->flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW |
		TTY_DRIVER_DYNAMIC_DEV | TTY_DRIVER_DEVPTS_MEM;
	pts_driver->other = ptm_driver;
	tty_set_operations(pts_driver, &pty_unix98_ops);

	if (tty_register_driver(ptm_driver))
		panic("Couldn't register Unix98 ptm driver");
	if (tty_register_driver(pts_driver))
		panic("Couldn't register Unix98 pts driver");

	register_sysctl_table(pty_root_table);

	/* Now create the /dev/ptmx special device */
	tty_default_fops(&ptmx_fops);
	ptmx_fops.open = ptmx_open;

	cdev_init(&ptmx_cdev, &ptmx_fops);
	if (cdev_add(&ptmx_cdev, MKDEV(TTYAUX_MAJOR, 2), 1) ||
	    register_chrdev_region(MKDEV(TTYAUX_MAJOR, 2), 1, "/dev/ptmx") < 0)
		panic("Couldn't register /dev/ptmx driver\n");
	device_create(tty_class, NULL, MKDEV(TTYAUX_MAJOR, 2), NULL, "ptmx");
}

#else
static inline void unix98_pty_init(void) { }
#endif

static int __init pty_init(void)
{
	legacy_pty_init();
	unix98_pty_init();
	return 0;
}
module_init(pty_init);
