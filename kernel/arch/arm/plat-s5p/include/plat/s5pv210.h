

/* Common init code for S5PV210 related SoCs */

extern void s5pv210_common_init_uarts(struct s3c2410_uartcfg *cfg, int no);
extern void s5pv210_register_clocks(void);
extern void s5pv210_setup_clocks(void);

#ifdef CONFIG_CPU_S5PV210

extern  int s5pv210_init(void);
extern void s5pv210_init_irq(void);
extern void s5pv210_map_io(void);
extern void s5pv210_init_clocks(int xtal);

#define s5pv210_init_uarts s5pv210_common_init_uarts

#else
#define s5pv210_init_clocks NULL
#define s5pv210_init_uarts NULL
#define s5pv210_map_io NULL
#define s5pv210_init NULL
#endif
