
#ifndef _ASM_POWERPC_ISERIES_ALPACA_H
#define _ASM_POWERPC_ISERIES_ALPACA_H

struct alpaca {
	struct lppaca *lppaca_ptr;	/* Pointer to LpPaca for PLIC */
	const void *reg_save_ptr;	/* Pointer to LpRegSave for PLIC */
};

#endif /* _ASM_POWERPC_ISERIES_ALPACA_H */
