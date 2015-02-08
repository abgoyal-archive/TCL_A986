

#ifndef _SPARC_MPMBOX_H
#define _SPARC_MPMBOX_H



/* The CPU is executing code in the kernel. */
#define MAILBOX_ISRUNNING     0xf0

#define MAILBOX_EXIT          0xfb

#define MAILBOX_GOSPIN        0xfc

#define MAILBOX_BPT_SPIN      0xfd

#define MAILBOX_WDOG_STOP     0xfe

#ifndef __ASSEMBLY__

/* Handy macro's to determine a cpu's state. */

/* Is the cpu still in Power On Self Test? */
#define MBOX_POST_P(letter)  ((letter) >= 0x00 && (letter) <= 0x7f)

/* Is the cpu at the 'ok' prompt of the PROM? */
#define MBOX_PROMPROMPT_P(letter) ((letter) >= 0x80 && (letter) <= 0x8f)

/* Is the cpu spinning in the PROM? */
#define MBOX_PROMSPIN_P(letter) ((letter) >= 0x90 && (letter) <= 0xef)

/* Sanity check... This is junk mail, throw it out. */
#define MBOX_BOGON_P(letter) ((letter) >= 0xf1 && (letter) <= 0xfa)

/* Is the cpu actively running an application/kernel-code? */
#define MBOX_RUNNING_P(letter) ((letter) == MAILBOX_ISRUNNING)

#endif /* !(__ASSEMBLY__) */

#endif /* !(_SPARC_MPMBOX_H) */
