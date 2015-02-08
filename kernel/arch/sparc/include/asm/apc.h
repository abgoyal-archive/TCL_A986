

#ifndef _SPARC_APC_H
#define _SPARC_APC_H

#include <linux/ioctl.h>

#define APC_IOC	'A'

#define APCIOCGFANCTL _IOR(APC_IOC, 0x00, int)	/* Get fan speed	*/
#define APCIOCSFANCTL _IOW(APC_IOC, 0x01, int)	/* Set fan speed	*/

#define APCIOCGCPWR   _IOR(APC_IOC, 0x02, int)	/* Get CPOWER state	*/
#define APCIOCSCPWR   _IOW(APC_IOC, 0x03, int)	/* Set CPOWER state	*/

#define APCIOCGBPORT   _IOR(APC_IOC, 0x04, int)	/* Get BPORT state 	*/
#define APCIOCSBPORT   _IOW(APC_IOC, 0x05, int)	/* Set BPORT state	*/

#define APC_IDLE_REG	0x00
#define APC_FANCTL_REG	0x20
#define APC_CPOWER_REG	0x24
#define APC_BPORT_REG	0x30

#define APC_REGMASK		0x01
#define APC_BPMASK		0x03

#define APC_IDLE_ON		0x01

#define APC_FANCTL_HI	0x00	/* Fan speed high	*/
#define APC_FANCTL_LO	0x01	/* Fan speed low	*/

#define APC_CPOWER_ON	0x00	/* Conv power on	*/
#define APC_CPOWER_OFF	0x01	/* Conv power off	*/

#define APC_BPORT_A		0x01	/* Bit Port A		*/
#define APC_BPORT_B		0x02	/* Bit Port B		*/

#endif /* !(_SPARC_APC_H) */
