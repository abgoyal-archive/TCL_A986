


#include <linux/sched.h>
#include <linux/syscalls.h>
#include <asm/errno.h>
#include <asm/ioctl.h>
#include <asm/termios.h>
#include <asm/uaccess.h>

static int hpux_ioctl_t(int fd, unsigned long cmd, unsigned long arg)
{
	int result = -EOPNOTSUPP;
	int nr = _IOC_NR(cmd);
	switch (nr) {
	case 106:
		result = sys_ioctl(fd, TIOCSWINSZ, arg);
		break;
	case 107:
		result = sys_ioctl(fd, TIOCGWINSZ, arg);
		break;
	}
	return result;
}

int hpux_ioctl(int fd, unsigned long cmd, unsigned long arg)
{
	int result = -EOPNOTSUPP;
	int type = _IOC_TYPE(cmd);
	switch (type) {
	case 'T':
		/* Our structures are now compatible with HPUX's */
		result = sys_ioctl(fd, cmd, arg);
		break;
	case 't':
		result = hpux_ioctl_t(fd, cmd, arg);
		break;
	}
	return result;
}
