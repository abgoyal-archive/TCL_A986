
#ifndef __ASM_SH_CPU_SH5_CACHE_H
#define __ASM_SH_CPU_SH5_CACHE_H


#define L1_CACHE_SHIFT		5

/* Valid and Dirty bits */
#define SH_CACHE_VALID		(1LL<<0)
#define SH_CACHE_UPDATED	(1LL<<57)

/* Unimplemented compat bits.. */
#define SH_CACHE_COMBINED	0
#define SH_CACHE_ASSOC		0

/* Cache flags */
#define SH_CACHE_MODE_WT	(1LL<<0)
#define SH_CACHE_MODE_WB	(1LL<<1)

#define ICCR_BASE	0x01600000	/* Instruction Cache Control Register */
#define ICCR_REG0	0		/* Register 0 offset */
#define ICCR_REG1	1		/* Register 1 offset */
#define ICCR0		ICCR_BASE+ICCR_REG0
#define ICCR1		ICCR_BASE+ICCR_REG1

#define ICCR0_OFF	0x0		/* Set ICACHE off */
#define ICCR0_ON	0x1		/* Set ICACHE on */
#define ICCR0_ICI	0x2		/* Invalidate all in IC */

#define ICCR1_NOLOCK	0x0		/* Set No Locking */

#define OCCR_BASE	0x01E00000	/* Operand Cache Control Register */
#define OCCR_REG0	0		/* Register 0 offset */
#define OCCR_REG1	1		/* Register 1 offset */
#define OCCR0		OCCR_BASE+OCCR_REG0
#define OCCR1		OCCR_BASE+OCCR_REG1

#define OCCR0_OFF	0x0		/* Set OCACHE off */
#define OCCR0_ON	0x1		/* Set OCACHE on */
#define OCCR0_OCI	0x2		/* Invalidate all in OC */
#define OCCR0_WT	0x4		/* Set OCACHE in WT Mode */
#define OCCR0_WB	0x0		/* Set OCACHE in WB Mode */

#define OCCR1_NOLOCK	0x0		/* Set No Locking */


/* Instruction cache */
#define CACHE_IC_ADDRESS_ARRAY 0x01000000

/* Operand Cache */
#define CACHE_OC_ADDRESS_ARRAY 0x01800000


#define CACHE_OC_N_SYNBITS  1               /* Number of synonym bits */
#define CACHE_OC_SYN_SHIFT  12
/* Mask to select synonym bit(s) */
#define CACHE_OC_SYN_MASK   (((1UL<<CACHE_OC_N_SYNBITS)-1)<<CACHE_OC_SYN_SHIFT)


#endif /* __ASM_SH_CPU_SH5_CACHE_H */
