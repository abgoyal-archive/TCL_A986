
/* Portions copyright Broadcom 2008 */
#ifndef __ASM_ARCH_BCMRING_H
#define __ASM_ARCH_BCMRING_H

void __init bcmring_amba_init(void);
void __init bcmring_map_io(void);
void __init bcmring_init_irq(void);

extern struct sys_timer bcmring_timer;
#endif
