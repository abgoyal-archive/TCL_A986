
#ifndef __ASM_MIPS_DEC_KN05_H
#define __ASM_MIPS_DEC_KN05_H

#include <asm/dec/ioasic_addrs.h>

#define KN4K_SLOT_BASE	0x1fc00000

#define KN4K_MB_ROM	(0*IOASIC_SLOT_SIZE)	/* KN05/KN04 card ROM */
#define KN4K_IOCTL	(1*IOASIC_SLOT_SIZE)	/* I/O ASIC */
#define KN4K_ESAR	(2*IOASIC_SLOT_SIZE)	/* LANCE MAC address chip */
#define KN4K_LANCE	(3*IOASIC_SLOT_SIZE)	/* LANCE Ethernet */
#define KN4K_MB_INT	(4*IOASIC_SLOT_SIZE)	/* MB interrupt register */
#define KN4K_MB_EA	(5*IOASIC_SLOT_SIZE)	/* MB error address? */
#define KN4K_MB_EC	(6*IOASIC_SLOT_SIZE)	/* MB error ??? */
#define KN4K_MB_CSR	(7*IOASIC_SLOT_SIZE)	/* MB control & status */
#define KN4K_RES_08	(8*IOASIC_SLOT_SIZE)	/* unused? */
#define KN4K_RES_09	(9*IOASIC_SLOT_SIZE)	/* unused? */
#define KN4K_RES_10	(10*IOASIC_SLOT_SIZE)	/* unused? */
#define KN4K_RES_11	(11*IOASIC_SLOT_SIZE)	/* unused? */
#define KN4K_SCSI	(12*IOASIC_SLOT_SIZE)	/* ASC SCSI */
#define KN4K_RES_13	(13*IOASIC_SLOT_SIZE)	/* unused? */
#define KN4K_RES_14	(14*IOASIC_SLOT_SIZE)	/* unused? */
#define KN4K_RES_15	(15*IOASIC_SLOT_SIZE)	/* unused? */

#define KN4K_MB_INT_TC		(1<<0)		/* TURBOchannel? */
#define KN4K_MB_INT_RTC		(1<<1)		/* RTC? */
#define KN4K_MB_INT_MT		(1<<3)		/* I/O ASIC cascade */

#define KN4K_MB_CSR_PF		(1<<0)		/* PreFetching enable? */
#define KN4K_MB_CSR_F		(1<<1)		/* ??? */
#define KN4K_MB_CSR_ECC		(0xff<<2)	/* ??? */
#define KN4K_MB_CSR_OD		(1<<10)		/* ??? */
#define KN4K_MB_CSR_CP		(1<<11)		/* ??? */
#define KN4K_MB_CSR_UNC		(1<<12)		/* ??? */
#define KN4K_MB_CSR_IM		(1<<13)		/* ??? */
#define KN4K_MB_CSR_NC		(1<<14)		/* ??? */
#define KN4K_MB_CSR_EE		(1<<15)		/* (bus) Exception Enable? */
#define KN4K_MB_CSR_MSK		(0x1f<<16)	/* CPU Int[4:0] mask */
#define KN4K_MB_CSR_FW		(1<<21)		/* ??? */
#define KN4K_MB_CSR_W		(1<<31)		/* ??? */

#endif /* __ASM_MIPS_DEC_KN05_H */
