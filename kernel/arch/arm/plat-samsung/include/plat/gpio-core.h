

#define GPIOCON_OFF	(0x00)
#define GPIODAT_OFF	(0x04)

#define con_4bit_shift(__off) ((__off) * 4)


struct s3c_gpio_chip;

struct s3c_gpio_pm {
	void (*save)(struct s3c_gpio_chip *chip);
	void (*resume)(struct s3c_gpio_chip *chip);
};

struct s3c_gpio_cfg;

struct s3c_gpio_chip {
	struct gpio_chip	chip;
	struct s3c_gpio_cfg	*config;
	struct s3c_gpio_pm	*pm;
	void __iomem		*base;
	spinlock_t		 lock;
#ifdef CONFIG_PM
	u32			pm_save[4];
#endif
};

static inline struct s3c_gpio_chip *to_s3c_gpio(struct gpio_chip *gpc)
{
	return container_of(gpc, struct s3c_gpio_chip, chip);
}

extern void s3c_gpiolib_add(struct s3c_gpio_chip *chip);


extern void samsung_gpiolib_add_4bit_chips(struct s3c_gpio_chip *chip,
					   int nr_chips);
extern void samsung_gpiolib_add_4bit2_chips(struct s3c_gpio_chip *chip,
					    int nr_chips);

extern void samsung_gpiolib_add_4bit(struct s3c_gpio_chip *chip);
extern void samsung_gpiolib_add_4bit2(struct s3c_gpio_chip *chip);

/* exported for core SoC support to change */
extern struct s3c_gpio_cfg s3c24xx_gpiocfg_default;

#ifdef CONFIG_S3C_GPIO_TRACK
extern struct s3c_gpio_chip *s3c_gpios[S3C_GPIO_END];

static inline struct s3c_gpio_chip *s3c_gpiolib_getchip(unsigned int chip)
{
	return (chip < S3C_GPIO_END) ? s3c_gpios[chip] : NULL;
}
#else
/* machine specific code should provide s3c_gpiolib_getchip */

#include <mach/gpio-track.h>

static inline void s3c_gpiolib_track(struct s3c_gpio_chip *chip) { }
#endif

#ifdef CONFIG_PM
extern struct s3c_gpio_pm s3c_gpio_pm_1bit;
extern struct s3c_gpio_pm s3c_gpio_pm_2bit;
extern struct s3c_gpio_pm s3c_gpio_pm_4bit;
#define __gpio_pm(x) x
#else
#define s3c_gpio_pm_1bit NULL
#define s3c_gpio_pm_2bit NULL
#define s3c_gpio_pm_4bit NULL
#define __gpio_pm(x) NULL

#endif /* CONFIG_PM */

/* locking wrappers to deal with multiple access to the same gpio bank */
#define s3c_gpio_lock(_oc, _fl) spin_lock_irqsave(&(_oc)->lock, _fl)
#define s3c_gpio_unlock(_oc, _fl) spin_unlock_irqrestore(&(_oc)->lock, _fl)
