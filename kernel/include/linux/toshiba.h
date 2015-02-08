

#ifndef _LINUX_TOSHIBA_H
#define _LINUX_TOSHIBA_H

#define TOSH_PROC "/proc/toshiba"
#define TOSH_DEVICE "/dev/toshiba"
#define TOSH_SMM _IOWR('t', 0x90, int)	/* broken: meant 24 bytes */

typedef struct {
	unsigned int eax;
	unsigned int ebx __attribute__ ((packed));
	unsigned int ecx __attribute__ ((packed));
	unsigned int edx __attribute__ ((packed));
	unsigned int esi __attribute__ ((packed));
	unsigned int edi __attribute__ ((packed));
} SMMRegisters;

int tosh_smm(SMMRegisters *regs);

#endif
