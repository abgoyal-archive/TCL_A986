

#ifndef ASM_BEAT_PIC_H
#define ASM_BEAT_PIC_H
#ifdef __KERNEL__

extern void beatic_init_IRQ(void);
extern unsigned int beatic_get_irq(void);
extern void beatic_cause_IPI(int cpu, int mesg);
extern void beatic_request_IPIs(void);
extern void beatic_setup_cpu(int);
extern void beatic_deinit_IRQ(void);

#endif
#endif /* ASM_BEAT_PIC_H */
