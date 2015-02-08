
#ifndef _SPARC_AUXIO_H
#define _SPARC_AUXIO_H

#include <asm/system.h>
#include <asm/vaddrs.h>

#define AUXIO_ORMEIN      0xf0    /* All writes must set these bits. */
#define AUXIO_ORMEIN4M    0xc0    /* sun4m - All writes must set these bits. */
#define AUXIO_FLPY_DENS   0x20    /* Floppy density, high if set. Read only. */
#define AUXIO_FLPY_DCHG   0x10    /* A disk change occurred.  Read only. */
#define AUXIO_EDGE_ON     0x10    /* sun4m - On means Jumper block is in. */
#define AUXIO_FLPY_DSEL   0x08    /* Drive select/start-motor. Write only. */
#define AUXIO_LINK_TEST   0x08    /* sun4m - On means TPE Carrier detect. */

/* Set the following to one, then zero, after doing a pseudo DMA transfer. */
#define AUXIO_FLPY_TCNT   0x04    /* Floppy terminal count. Write only. */

/* Set the following to zero to eject the floppy. */
#define AUXIO_FLPY_EJCT   0x02    /* Eject floppy disk.  Write only. */
#define AUXIO_LED         0x01    /* On if set, off if unset. Read/Write */

#ifndef __ASSEMBLY__

extern void set_auxio(unsigned char bits_on, unsigned char bits_off);
extern unsigned char get_auxio(void); /* .../asm/floppy.h */


#define AUXIO_LTE_ON    1
#define AUXIO_LTE_OFF   0

#define auxio_set_lte(on) \
do { \
	if(on) { \
		set_auxio(AUXIO_LINK_TEST, 0); \
	} else { \
		set_auxio(0, AUXIO_LINK_TEST); \
	} \
} while (0)

#define AUXIO_LED_ON    1
#define AUXIO_LED_OFF   0

#define auxio_set_led(on) \
do { \
	if(on) { \
		set_auxio(AUXIO_LED, 0); \
	} else { \
		set_auxio(0, AUXIO_LED); \
	} \
} while (0)

#endif /* !(__ASSEMBLY__) */


/* AUXIO2 (Power Off Control) */
extern __volatile__ unsigned char * auxio_power_register;

#define	AUXIO_POWER_DETECT_FAILURE	32
#define	AUXIO_POWER_CLEAR_FAILURE	2
#define	AUXIO_POWER_OFF			1


#endif /* !(_SPARC_AUXIO_H) */
