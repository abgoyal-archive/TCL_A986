

extern unsigned long h720x_gettimeoffset(void);
extern void __init h720x_init_irq(void);
extern void __init h720x_map_io(void);

#ifdef CONFIG_ARCH_H7202
extern struct sys_timer h7202_timer;
extern void __init init_hw_h7202(void);
extern void __init h7202_init_irq(void);
extern void __init h7202_init_time(void);
#endif

#ifdef CONFIG_ARCH_H7201
extern struct sys_timer h7201_timer;
#endif
