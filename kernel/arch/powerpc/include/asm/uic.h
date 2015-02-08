
#ifndef _ASM_POWERPC_UIC_H
#define _ASM_POWERPC_UIC_H

#ifdef __KERNEL__

extern void __init uic_init_tree(void);
extern unsigned int uic_get_irq(void);

#endif /* __KERNEL__ */
#endif /* _ASM_POWERPC_UIC_H */
