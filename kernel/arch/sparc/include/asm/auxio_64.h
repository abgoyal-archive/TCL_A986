
#ifndef _SPARC64_AUXIO_H
#define _SPARC64_AUXIO_H

#define AUXIO_AUX1_MASK		0xc0 /* Mask bits 		*/
#define AUXIO_AUX1_FDENS	0x20 /* Floppy Density Sense	*/
#define AUXIO_AUX1_LTE 		0x08 /* Link Test Enable 	*/
#define AUXIO_AUX1_MMUX		0x04 /* Monitor/Mouse Mux	*/
#define AUXIO_AUX1_FTCNT	0x02 /* Terminal Count, 	*/
#define AUXIO_AUX1_LED		0x01 /* System LED		*/

#define AUXIO_AUX2_MASK		0xdc /* Mask Bits		*/
#define AUXIO_AUX2_PFAILDET	0x20 /* Power Fail Detect	*/
#define AUXIO_AUX2_PFAILCLR 	0x02 /* Clear Pwr Fail Det Intr	*/
#define AUXIO_AUX2_PWR_OFF	0x01 /* Power Off		*/

#define AUXIO_PCIO_LED		0x01 /* System LED 		*/

#define AUXIO_PCIO_CPWR_OFF	0x02 /* Courtesy Power Off	*/
#define AUXIO_PCIO_SPWR_OFF	0x01 /* System Power Off	*/

#ifndef __ASSEMBLY__

extern void __iomem *auxio_register;

#define AUXIO_LTE_ON	1
#define AUXIO_LTE_OFF	0

extern void auxio_set_lte(int on);

#define AUXIO_LED_ON	1
#define AUXIO_LED_OFF	0

extern void auxio_set_led(int on);

#endif /* ifndef __ASSEMBLY__ */

#endif /* !(_SPARC64_AUXIO_H) */
