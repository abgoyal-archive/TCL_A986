


#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <asm/pci.h>
#include <asm/io.h>
#include <linux/init.h>
#include <asm/termios.h>
#include <asm/ioctls.h>
#include <linux/ioctl.h>
#include <linux/fcntl.h>

#define	DEFAULT_PORT 	"/dev/ttyS0"	/* Port to open */
#define	TXX		0 		/* Dummy loop for spinning */

#define	BLOCK_SEL	0x00
#define	SLAVE_ADDR	0xa0
#define	READ_BIT	0x01
#define	WRITE_BIT	0x00
#define	R_HEADER	SLAVE_ADDR + BLOCK_SEL + READ_BIT
#define	W_HEADER	SLAVE_ADDR + BLOCK_SEL + WRITE_BIT

#define	vcc_off		(ioctl(fd, TIOCSBRK, 0))
#define	vcc_on		(ioctl(fd, TIOCCBRK, 0))
#define	sda_hi		(ioctl(fd, TIOCMBIS, &dtr))
#define	sda_lo		(ioctl(fd, TIOCMBIC, &dtr))
#define	scl_lo		(ioctl(fd, TIOCMBIC, &rts))
#define	scl_hi		(ioctl(fd, TIOCMBIS, &rts))

const char rts = TIOCM_RTS;
const char dtr = TIOCM_DTR;
int fd;
