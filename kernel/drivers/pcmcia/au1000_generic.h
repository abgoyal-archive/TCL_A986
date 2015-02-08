
#ifndef __ASM_AU1000_PCMCIA_H
#define __ASM_AU1000_PCMCIA_H

/* include the world */

#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/ss.h>
#include <pcmcia/cistpl.h>
#include "cs_internal.h"

#define AU1000_PCMCIA_POLL_PERIOD    (2*HZ)
#define AU1000_PCMCIA_IO_SPEED       (255)
#define AU1000_PCMCIA_MEM_SPEED      (300)

#define AU1X_SOCK0_IO        0xF00000000ULL
#define AU1X_SOCK0_PHYS_ATTR 0xF40000000ULL
#define AU1X_SOCK0_PHYS_MEM  0xF80000000ULL

#if defined(CONFIG_MIPS_PB1000)
#define AU1X_SOCK1_IO        0xF08000000ULL
#define AU1X_SOCK1_PHYS_ATTR 0xF48000000ULL
#define AU1X_SOCK1_PHYS_MEM  0xF88000000ULL
#endif

struct pcmcia_state {
  unsigned detect: 1,
            ready: 1,
           wrprot: 1,
	     bvd1: 1,
	     bvd2: 1,
            vs_3v: 1,
            vs_Xv: 1;
};

struct pcmcia_configure {
  unsigned sock: 8,
            vcc: 8,
            vpp: 8,
         output: 1,
        speaker: 1,
          reset: 1;
};

struct pcmcia_irqs {
	int sock;
	int irq;
	const char *str;
};


struct au1000_pcmcia_socket {
	struct pcmcia_socket socket;

	/*
	 * Info from low level handler
	 */
	struct device		*dev;
	unsigned int		nr;
	unsigned int		irq;

	/*
	 * Core PCMCIA state
	 */
	struct pcmcia_low_level *ops;

	unsigned int 		status;
	socket_state_t		cs_state;

	unsigned short		spd_io[MAX_IO_WIN];
	unsigned short		spd_mem[MAX_WIN];
	unsigned short		spd_attr[MAX_WIN];

	struct resource		res_skt;
	struct resource		res_io;
	struct resource		res_mem;
	struct resource		res_attr;

	void *                 	virt_io;
	unsigned int		phys_io;
	unsigned int           	phys_attr;
	unsigned int           	phys_mem;
	unsigned short        	speed_io, speed_attr, speed_mem;

	unsigned int		irq_state;

	struct timer_list	poll_timer;
};

struct pcmcia_low_level {
	struct module *owner;

	int (*hw_init)(struct au1000_pcmcia_socket *);
	void (*hw_shutdown)(struct au1000_pcmcia_socket *);

	void (*socket_state)(struct au1000_pcmcia_socket *, struct pcmcia_state *);
	int (*configure_socket)(struct au1000_pcmcia_socket *, struct socket_state_t *);

	/*
	 * Enable card status IRQs on (re-)initialisation.  This can
	 * be called at initialisation, power management event, or
	 * pcmcia event.
	 */
	void (*socket_init)(struct au1000_pcmcia_socket *);

	/*
	 * Disable card status IRQs and PCMCIA bus on suspend.
	 */
	void (*socket_suspend)(struct au1000_pcmcia_socket *);
};

extern int au1x_board_init(struct device *dev);

#endif /* __ASM_AU1000_PCMCIA_H */
