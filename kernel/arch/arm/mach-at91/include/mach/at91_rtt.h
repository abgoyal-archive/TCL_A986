

#ifndef AT91_RTT_H
#define AT91_RTT_H

#define AT91_RTT_MR		0x00			/* Real-time Mode Register */
#define		AT91_RTT_RTPRES		(0xffff << 0)		/* Real-time Timer Prescaler Value */
#define		AT91_RTT_ALMIEN		(1 << 16)		/* Alarm Interrupt Enable */
#define		AT91_RTT_RTTINCIEN	(1 << 17)		/* Real Time Timer Increment Interrupt Enable */
#define		AT91_RTT_RTTRST		(1 << 18)		/* Real Time Timer Restart */

#define AT91_RTT_AR		0x04			/* Real-time Alarm Register */
#define		AT91_RTT_ALMV		(0xffffffff)		/* Alarm Value */

#define AT91_RTT_VR		0x08			/* Real-time Value Register */
#define		AT91_RTT_CRTV		(0xffffffff)		/* Current Real-time Value */

#define AT91_RTT_SR		0x0c			/* Real-time Status Register */
#define		AT91_RTT_ALMS		(1 << 0)		/* Real-time Alarm Status */
#define		AT91_RTT_RTTINC		(1 << 1)		/* Real-time Timer Increment */

#endif
