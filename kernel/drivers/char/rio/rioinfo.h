

#ifndef __rioinfo_h
#define __rioinfo_h

struct RioHostInfo {
	long location;		/* RIO Card Base I/O address */
	long vector;		/* RIO Card IRQ vector */
	int bus;		/* ISA/EISA/MCA/PCI */
	int mode;		/* pointer to host mode - INTERRUPT / POLLED */
	struct old_sgttyb
	*Sg;			/* pointer to default term characteristics */
};


/* Mode in rio device info */
#define INTERRUPTED_MODE	0x01	/* Interrupt is generated */
#define POLLED_MODE		0x02	/* No interrupt */
#define AUTO_MODE		0x03	/* Auto mode */

#define WORD_ACCESS_MODE	0x10	/* Word Access Mode */
#define BYTE_ACCESS_MODE	0x20	/* Byte Access Mode */


/* Bus type that RIO supports */
#define ISA_BUS			0x01	/* The card is ISA */
#define EISA_BUS		0x02	/* The card is EISA */
#define MCA_BUS			0x04	/* The card is MCA */
#define PCI_BUS			0x08	/* The card is PCI */


#define DEF_TERM_CHARACTERISTICS \
{ \
	B19200, B19200,				/* input and output speed */ \
	'H' - '@',				/* erase char */ \
	-1,					/* 2nd erase char */ \
	'U' - '@',				/* kill char */ \
	ECHO | CRMOD,				/* mode */ \
	'C' - '@',				/* interrupt character */ \
	'\\' - '@',				/* quit char */ \
	'Q' - '@',				/* start char */ \
	'S' - '@',				/* stop char */ \
	'D' - '@',				/* EOF */ \
	-1,					/* brk */ \
	(LCRTBS | LCRTERA | LCRTKIL | LCTLECH),	/* local mode word */ \
	'Z' - '@',				/* process stop */ \
	'Y' - '@',				/* delayed stop */ \
	'R' - '@',				/* reprint line */ \
	'O' - '@',				/* flush output */ \
	'W' - '@',				/* word erase */ \
	'V' - '@'				/* literal next char */ \
}

#endif				/* __rioinfo_h */
