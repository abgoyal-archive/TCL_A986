
#ifndef _ASM_SETUP_H
#define _ASM_SETUP_H

#ifdef __KERNEL__
extern void __init unit_setup(void);
extern void __init unit_init_IRQ(void);
#endif
#endif /* _ASM_SETUP_H */
