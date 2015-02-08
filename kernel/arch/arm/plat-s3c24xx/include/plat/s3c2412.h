

#ifdef CONFIG_CPU_S3C2412

extern  int s3c2412_init(void);

extern void s3c2412_map_io(void);

extern void s3c2412_init_uarts(struct s3c2410_uartcfg *cfg, int no);

extern void s3c2412_init_clocks(int xtal);

extern  int s3c2412_baseclk_add(void);
#else
#define s3c2412_init_clocks NULL
#define s3c2412_init_uarts NULL
#define s3c2412_map_io NULL
#define s3c2412_init NULL
#endif
