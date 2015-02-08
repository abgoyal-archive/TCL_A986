

#ifndef __PLAT_S3C_SDHCI_H
#define __PLAT_S3C_SDHCI_H __FILE__

struct platform_device;
struct mmc_host;
struct mmc_card;
struct mmc_ios;

struct s3c_sdhci_platdata {
	unsigned int	max_width;
	unsigned int	host_caps;

	char		**clocks;	/* set of clock sources */

	void	(*cfg_gpio)(struct platform_device *dev, int width);
	void	(*cfg_card)(struct platform_device *dev,
			    void __iomem *regbase,
			    struct mmc_ios *ios,
			    struct mmc_card *card);
};

extern void s3c_sdhci0_set_platdata(struct s3c_sdhci_platdata *pd);
extern void s3c_sdhci1_set_platdata(struct s3c_sdhci_platdata *pd);
extern void s3c_sdhci2_set_platdata(struct s3c_sdhci_platdata *pd);


extern struct s3c_sdhci_platdata s3c_hsmmc0_def_platdata;
extern struct s3c_sdhci_platdata s3c_hsmmc1_def_platdata;
extern struct s3c_sdhci_platdata s3c_hsmmc2_def_platdata;

/* Helper function availablity */

extern void s3c64xx_setup_sdhci0_cfg_gpio(struct platform_device *, int w);
extern void s3c64xx_setup_sdhci1_cfg_gpio(struct platform_device *, int w);
extern void s5pc100_setup_sdhci0_cfg_gpio(struct platform_device *, int w);
extern void s5pc100_setup_sdhci1_cfg_gpio(struct platform_device *, int w);
extern void s5pc100_setup_sdhci2_cfg_gpio(struct platform_device *, int w);
extern void s3c64xx_setup_sdhci2_cfg_gpio(struct platform_device *, int w);
extern void s5pv210_setup_sdhci0_cfg_gpio(struct platform_device *, int w);
extern void s5pv210_setup_sdhci1_cfg_gpio(struct platform_device *, int w);
extern void s5pv210_setup_sdhci2_cfg_gpio(struct platform_device *, int w);

/* S3C6400 SDHCI setup */

#ifdef CONFIG_S3C64XX_SETUP_SDHCI
extern char *s3c64xx_hsmmc_clksrcs[4];

#ifdef CONFIG_S3C_DEV_HSMMC
extern void s3c6400_setup_sdhci_cfg_card(struct platform_device *dev,
					 void __iomem *r,
					 struct mmc_ios *ios,
					 struct mmc_card *card);

static inline void s3c6400_default_sdhci0(void)
{
	s3c_hsmmc0_def_platdata.clocks = s3c64xx_hsmmc_clksrcs;
	s3c_hsmmc0_def_platdata.cfg_gpio = s3c64xx_setup_sdhci0_cfg_gpio;
	s3c_hsmmc0_def_platdata.cfg_card = s3c6400_setup_sdhci_cfg_card;
}

#else
static inline void s3c6400_default_sdhci0(void) { }
#endif  /* CONFIG_S3C_DEV_HSMMC */

#ifdef CONFIG_S3C_DEV_HSMMC1
static inline void s3c6400_default_sdhci1(void)
{
	s3c_hsmmc1_def_platdata.clocks = s3c64xx_hsmmc_clksrcs;
	s3c_hsmmc1_def_platdata.cfg_gpio = s3c64xx_setup_sdhci1_cfg_gpio;
	s3c_hsmmc1_def_platdata.cfg_card = s3c6400_setup_sdhci_cfg_card;
}
#else
static inline void s3c6400_default_sdhci1(void) { }
#endif /* CONFIG_S3C_DEV_HSMMC1 */

#ifdef CONFIG_S3C_DEV_HSMMC2
static inline void s3c6400_default_sdhci2(void)
{
	s3c_hsmmc2_def_platdata.clocks = s3c64xx_hsmmc_clksrcs;
	s3c_hsmmc2_def_platdata.cfg_gpio = s3c64xx_setup_sdhci2_cfg_gpio;
	s3c_hsmmc2_def_platdata.cfg_card = s3c6400_setup_sdhci_cfg_card;
}
#else
static inline void s3c6400_default_sdhci2(void) { }
#endif /* CONFIG_S3C_DEV_HSMMC2 */

/* S3C6410 SDHCI setup */

extern void s3c6410_setup_sdhci_cfg_card(struct platform_device *dev,
					 void __iomem *r,
					 struct mmc_ios *ios,
					 struct mmc_card *card);

#ifdef CONFIG_S3C_DEV_HSMMC
static inline void s3c6410_default_sdhci0(void)
{
	s3c_hsmmc0_def_platdata.clocks = s3c64xx_hsmmc_clksrcs;
	s3c_hsmmc0_def_platdata.cfg_gpio = s3c64xx_setup_sdhci0_cfg_gpio;
	s3c_hsmmc0_def_platdata.cfg_card = s3c6410_setup_sdhci_cfg_card;
}
#else
static inline void s3c6410_default_sdhci0(void) { }
#endif /* CONFIG_S3C_DEV_HSMMC */

#ifdef CONFIG_S3C_DEV_HSMMC1
static inline void s3c6410_default_sdhci1(void)
{
	s3c_hsmmc1_def_platdata.clocks = s3c64xx_hsmmc_clksrcs;
	s3c_hsmmc1_def_platdata.cfg_gpio = s3c64xx_setup_sdhci1_cfg_gpio;
	s3c_hsmmc1_def_platdata.cfg_card = s3c6410_setup_sdhci_cfg_card;
}
#else
static inline void s3c6410_default_sdhci1(void) { }
#endif /* CONFIG_S3C_DEV_HSMMC1 */

#ifdef CONFIG_S3C_DEV_HSMMC2
static inline void s3c6410_default_sdhci2(void)
{
	s3c_hsmmc2_def_platdata.clocks = s3c64xx_hsmmc_clksrcs;
	s3c_hsmmc2_def_platdata.cfg_gpio = s3c64xx_setup_sdhci2_cfg_gpio;
	s3c_hsmmc2_def_platdata.cfg_card = s3c6410_setup_sdhci_cfg_card;
}
#else
static inline void s3c6410_default_sdhci2(void) { }
#endif /* CONFIG_S3C_DEV_HSMMC2 */

#else
static inline void s3c6410_default_sdhci0(void) { }
static inline void s3c6410_default_sdhci1(void) { }
static inline void s3c6410_default_sdhci2(void) { }
static inline void s3c6400_default_sdhci0(void) { }
static inline void s3c6400_default_sdhci1(void) { }
static inline void s3c6400_default_sdhci2(void) { }

#endif /* CONFIG_S3C64XX_SETUP_SDHCI */

/* S5PC100 SDHCI setup */

#ifdef CONFIG_S5PC100_SETUP_SDHCI
extern char *s5pc100_hsmmc_clksrcs[4];

extern void s5pc100_setup_sdhci0_cfg_card(struct platform_device *dev,
					   void __iomem *r,
					   struct mmc_ios *ios,
					   struct mmc_card *card);

#ifdef CONFIG_S3C_DEV_HSMMC
static inline void s5pc100_default_sdhci0(void)
{
	s3c_hsmmc0_def_platdata.clocks = s5pc100_hsmmc_clksrcs;
	s3c_hsmmc0_def_platdata.cfg_gpio = s5pc100_setup_sdhci0_cfg_gpio;
	s3c_hsmmc0_def_platdata.cfg_card = s5pc100_setup_sdhci0_cfg_card;
}
#else
static inline void s5pc100_default_sdhci0(void) { }
#endif /* CONFIG_S3C_DEV_HSMMC */

#ifdef CONFIG_S3C_DEV_HSMMC1
static inline void s5pc100_default_sdhci1(void)
{
	s3c_hsmmc1_def_platdata.clocks = s5pc100_hsmmc_clksrcs;
	s3c_hsmmc1_def_platdata.cfg_gpio = s5pc100_setup_sdhci1_cfg_gpio;
	s3c_hsmmc1_def_platdata.cfg_card = s5pc100_setup_sdhci0_cfg_card;
}
#else
static inline void s5pc100_default_sdhci1(void) { }
#endif /* CONFIG_S3C_DEV_HSMMC1 */

#ifdef CONFIG_S3C_DEV_HSMMC2
static inline void s5pc100_default_sdhci2(void)
{
	s3c_hsmmc2_def_platdata.clocks = s5pc100_hsmmc_clksrcs;
	s3c_hsmmc2_def_platdata.cfg_gpio = s5pc100_setup_sdhci2_cfg_gpio;
	s3c_hsmmc2_def_platdata.cfg_card = s5pc100_setup_sdhci0_cfg_card;
}
#else
static inline void s5pc100_default_sdhci2(void) { }
#endif /* CONFIG_S3C_DEV_HSMMC1 */


#else
static inline void s5pc100_default_sdhci0(void) { }
static inline void s5pc100_default_sdhci1(void) { }
static inline void s5pc100_default_sdhci2(void) { }
#endif /* CONFIG_S5PC100_SETUP_SDHCI */


/* S5PC110 SDHCI setup */
#ifdef CONFIG_S5PV210_SETUP_SDHCI
extern char *s5pv210_hsmmc_clksrcs[4];

extern void s5pv210_setup_sdhci_cfg_card(struct platform_device *dev,
					   void __iomem *r,
					   struct mmc_ios *ios,
					   struct mmc_card *card);

#ifdef CONFIG_S3C_DEV_HSMMC
static inline void s5pv210_default_sdhci0(void)
{
	s3c_hsmmc0_def_platdata.clocks = s5pv210_hsmmc_clksrcs;
	s3c_hsmmc0_def_platdata.cfg_gpio = s5pv210_setup_sdhci0_cfg_gpio;
	s3c_hsmmc0_def_platdata.cfg_card = s5pv210_setup_sdhci_cfg_card;
}
#else
static inline void s5pv210_default_sdhci0(void) { }
#endif /* CONFIG_S3C_DEV_HSMMC */

#ifdef CONFIG_S3C_DEV_HSMMC1
static inline void s5pv210_default_sdhci1(void)
{
	s3c_hsmmc1_def_platdata.clocks = s5pv210_hsmmc_clksrcs;
	s3c_hsmmc1_def_platdata.cfg_gpio = s5pv210_setup_sdhci1_cfg_gpio;
	s3c_hsmmc1_def_platdata.cfg_card = s5pv210_setup_sdhci_cfg_card;
}
#else
static inline void s5pv210_default_sdhci1(void) { }
#endif /* CONFIG_S3C_DEV_HSMMC1 */

#ifdef CONFIG_S3C_DEV_HSMMC2
static inline void s5pv210_default_sdhci2(void)
{
	s3c_hsmmc2_def_platdata.clocks = s5pv210_hsmmc_clksrcs;
	s3c_hsmmc2_def_platdata.cfg_gpio = s5pv210_setup_sdhci2_cfg_gpio;
	s3c_hsmmc2_def_platdata.cfg_card = s5pv210_setup_sdhci_cfg_card;
}
#else
static inline void s5pv210_default_sdhci2(void) { }
#endif /* CONFIG_S3C_DEV_HSMMC2 */

#else
static inline void s5pv210_default_sdhci0(void) { }
static inline void s5pv210_default_sdhci1(void) { }
static inline void s5pv210_default_sdhci2(void) { }
#endif /* CONFIG_S5PC100_SETUP_SDHCI */




#endif /* __PLAT_S3C_SDHCI_H */
