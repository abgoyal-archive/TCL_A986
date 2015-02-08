
#ifndef _LINUX_TTY_DRIVER_H
#define _LINUX_TTY_DRIVER_H


#include <linux/fs.h>
#include <linux/list.h>
#include <linux/cdev.h>

struct tty_struct;
struct tty_driver;

struct tty_operations {
	struct tty_struct * (*lookup)(struct tty_driver *driver,
			struct inode *inode, int idx);
	int  (*install)(struct tty_driver *driver, struct tty_struct *tty);
	void (*remove)(struct tty_driver *driver, struct tty_struct *tty);
	int  (*open)(struct tty_struct * tty, struct file * filp);
	void (*close)(struct tty_struct * tty, struct file * filp);
	void (*shutdown)(struct tty_struct *tty);
	void (*cleanup)(struct tty_struct *tty);
	int  (*write)(struct tty_struct * tty,
		      const unsigned char *buf, int count);
	int  (*put_char)(struct tty_struct *tty, unsigned char ch);
	void (*flush_chars)(struct tty_struct *tty);
	int  (*write_room)(struct tty_struct *tty);
	int  (*chars_in_buffer)(struct tty_struct *tty);
	int  (*ioctl)(struct tty_struct *tty, struct file * file,
		    unsigned int cmd, unsigned long arg);
	long (*compat_ioctl)(struct tty_struct *tty, struct file * file,
			     unsigned int cmd, unsigned long arg);
	void (*set_termios)(struct tty_struct *tty, struct ktermios * old);
	void (*throttle)(struct tty_struct * tty);
	void (*unthrottle)(struct tty_struct * tty);
	void (*stop)(struct tty_struct *tty);
	void (*start)(struct tty_struct *tty);
	void (*hangup)(struct tty_struct *tty);
	int (*break_ctl)(struct tty_struct *tty, int state);
	void (*flush_buffer)(struct tty_struct *tty);
	void (*set_ldisc)(struct tty_struct *tty);
	void (*wait_until_sent)(struct tty_struct *tty, int timeout);
	void (*send_xchar)(struct tty_struct *tty, char ch);
	int (*tiocmget)(struct tty_struct *tty, struct file *file);
	int (*tiocmset)(struct tty_struct *tty, struct file *file,
			unsigned int set, unsigned int clear);
	int (*resize)(struct tty_struct *tty, struct winsize *ws);
	int (*set_termiox)(struct tty_struct *tty, struct termiox *tnew);
#ifdef CONFIG_CONSOLE_POLL
	int (*poll_init)(struct tty_driver *driver, int line, char *options);
	int (*poll_get_char)(struct tty_driver *driver, int line);
	void (*poll_put_char)(struct tty_driver *driver, int line, char ch);
#endif
	const struct file_operations *proc_fops;
};

struct tty_driver {
	int	magic;		/* magic number for this structure */
	struct kref kref;	/* Reference management */
	struct cdev cdev;
	struct module	*owner;
	const char	*driver_name;
	const char	*name;
	int	name_base;	/* offset of printed name */
	int	major;		/* major device number */
	int	minor_start;	/* start of minor device number */
	int	minor_num;	/* number of *possible* devices */
	int	num;		/* number of devices allocated */
	short	type;		/* type of tty driver */
	short	subtype;	/* subtype of tty driver */
	struct ktermios init_termios; /* Initial termios */
	int	flags;		/* tty driver flags */
	struct proc_dir_entry *proc_entry; /* /proc fs entry */
	struct tty_driver *other; /* only used for the PTY driver */

	/*
	 * Pointer to the tty data structures
	 */
	struct tty_struct **ttys;
	struct ktermios **termios;
	struct ktermios **termios_locked;
	void *driver_state;

	/*
	 * Driver methods
	 */

	const struct tty_operations *ops;
	struct list_head tty_drivers;
};

extern struct list_head tty_drivers;

extern struct tty_driver *alloc_tty_driver(int lines);
extern void put_tty_driver(struct tty_driver *driver);
extern void tty_set_operations(struct tty_driver *driver,
			const struct tty_operations *op);
extern struct tty_driver *tty_find_polling_driver(char *name, int *line);

extern void tty_driver_kref_put(struct tty_driver *driver);

static inline struct tty_driver *tty_driver_kref_get(struct tty_driver *d)
{
	kref_get(&d->kref);
	return d;
}

/* tty driver magic number */
#define TTY_DRIVER_MAGIC		0x5402

#define TTY_DRIVER_INSTALLED		0x0001
#define TTY_DRIVER_RESET_TERMIOS	0x0002
#define TTY_DRIVER_REAL_RAW		0x0004
#define TTY_DRIVER_DYNAMIC_DEV		0x0008
#define TTY_DRIVER_DEVPTS_MEM		0x0010
#define TTY_DRIVER_HARDWARE_BREAK	0x0020

/* tty driver types */
#define TTY_DRIVER_TYPE_SYSTEM		0x0001
#define TTY_DRIVER_TYPE_CONSOLE		0x0002
#define TTY_DRIVER_TYPE_SERIAL		0x0003
#define TTY_DRIVER_TYPE_PTY		0x0004
#define TTY_DRIVER_TYPE_SCC		0x0005	/* scc driver */
#define TTY_DRIVER_TYPE_SYSCONS		0x0006

/* system subtypes (magic, used by tty_io.c) */
#define SYSTEM_TYPE_TTY			0x0001
#define SYSTEM_TYPE_CONSOLE		0x0002
#define SYSTEM_TYPE_SYSCONS		0x0003
#define SYSTEM_TYPE_SYSPTMX		0x0004

/* pty subtypes (magic, used by tty_io.c) */
#define PTY_TYPE_MASTER			0x0001
#define PTY_TYPE_SLAVE			0x0002

/* serial subtype definitions */
#define SERIAL_TYPE_NORMAL	1

#endif /* #ifdef _LINUX_TTY_DRIVER_H */
