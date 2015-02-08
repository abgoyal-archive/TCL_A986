

/****************************************************************************/
/****************************************************************************/

#ifndef SECHW_INLINE_H
#define SECHW_INLINE_H

/****************************************************************************/
/****************************************************************************/
static inline void secHw_setSecure(uint32_t mask	/*  mask of type secHw_BLK_MASK_XXXXXX */
    ) {
	secHw_REGS_t *regp = (secHw_REGS_t *) MM_IO_BASE_TZPC;

	if (mask & 0x0000FFFF) {
		regp->reg[secHw_IDX_LS].setSecure = mask & 0x0000FFFF;
	}

	if (mask & 0xFFFF0000) {
		regp->reg[secHw_IDX_MS].setSecure = mask >> 16;
	}
}

/****************************************************************************/
/****************************************************************************/
static inline void secHw_setUnsecure(uint32_t mask	/*  mask of type secHw_BLK_MASK_XXXXXX */
    ) {
	secHw_REGS_t *regp = (secHw_REGS_t *) MM_IO_BASE_TZPC;

	if (mask & 0x0000FFFF) {
		regp->reg[secHw_IDX_LS].setUnsecure = mask & 0x0000FFFF;
	}
	if (mask & 0xFFFF0000) {
		regp->reg[secHw_IDX_MS].setUnsecure = mask >> 16;
	}
}

/****************************************************************************/
/****************************************************************************/
static inline uint32_t secHw_getStatus(void)
{
	secHw_REGS_t *regp = (secHw_REGS_t *) MM_IO_BASE_TZPC;

	return (regp->reg[1].status << 16) + regp->reg[0].status;
}

#endif /* SECHW_INLINE_H */
