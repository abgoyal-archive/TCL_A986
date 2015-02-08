

 /* Common init code for S5P6440 related SoCs */

extern void s5p6440_common_init_uarts(struct s3c2410_uartcfg *cfg, int no);
extern void s5p6440_register_clocks(void);
extern void s5p6440_setup_clocks(void);

#ifdef CONFIG_CPU_S5P6440

extern  int s5p6440_init(void);
extern void s5p6440_init_irq(void);
extern void s5p6440_map_io(void);
extern void s5p6440_init_clocks(int xtal);

#define s5p6440_init_uarts s5p6440_common_init_uarts

#else
#define s5p6440_init_clocks NULL
#define s5p6440_init_uarts NULL
#define s5p6440_map_io NULL
#define s5p6440_init NULL
#endif

/* S5P6440 timer */

extern struct sys_timer s5p6440_timer;
