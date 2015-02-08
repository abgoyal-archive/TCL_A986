

#ifndef _SPARC64_WATCHDOG_H
#define _SPARC64_WATCHDOG_H

#include <linux/watchdog.h>

#define WIOCSTART _IO (WATCHDOG_IOCTL_BASE, 10)		/* Start Timer		*/
#define WIOCSTOP  _IO (WATCHDOG_IOCTL_BASE, 11)		/* Stop Timer		*/
#define WIOCGSTAT _IOR(WATCHDOG_IOCTL_BASE, 12, int)/* Get Timer Status	*/

#define WD_FREERUN	0x01	/* timer is running, interrupts disabled	*/
#define WD_EXPIRED	0x02	/* timer has expired						*/
#define WD_RUNNING	0x04	/* timer is running, interrupts enabled		*/
#define WD_STOPPED	0x08	/* timer has not been started				*/
#define WD_SERVICED 0x10	/* timer interrupt was serviced				*/

#endif /* ifndef _SPARC64_WATCHDOG_H */

