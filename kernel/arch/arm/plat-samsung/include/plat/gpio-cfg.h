



#ifndef __PLAT_GPIO_CFG_H
#define __PLAT_GPIO_CFG_H __FILE__

typedef unsigned int __bitwise__ s3c_gpio_pull_t;
typedef unsigned int __bitwise__ s5p_gpio_drvstr_t;

/* forward declaration if gpio-core.h hasn't been included */
struct s3c_gpio_chip;

struct s3c_gpio_cfg {
	unsigned int	cfg_eint;

	s3c_gpio_pull_t	(*get_pull)(struct s3c_gpio_chip *chip, unsigned offs);
	int		(*set_pull)(struct s3c_gpio_chip *chip, unsigned offs,
				    s3c_gpio_pull_t pull);

	unsigned (*get_config)(struct s3c_gpio_chip *chip, unsigned offs);
	int	 (*set_config)(struct s3c_gpio_chip *chip, unsigned offs,
			       unsigned config);
};

#define S3C_GPIO_SPECIAL_MARK	(0xfffffff0)
#define S3C_GPIO_SPECIAL(x)	(S3C_GPIO_SPECIAL_MARK | (x))

/* Defines for generic pin configurations */
#define S3C_GPIO_INPUT	(S3C_GPIO_SPECIAL(0))
#define S3C_GPIO_OUTPUT	(S3C_GPIO_SPECIAL(1))
#define S3C_GPIO_SFN(x)	(S3C_GPIO_SPECIAL(x))

#define s3c_gpio_is_cfg_special(_cfg) \
	(((_cfg) & S3C_GPIO_SPECIAL_MARK) == S3C_GPIO_SPECIAL_MARK)

extern int s3c_gpio_cfgpin(unsigned int pin, unsigned int to);

extern unsigned s3c_gpio_getcfg(unsigned int pin);

#define S3C_GPIO_PULL_NONE	((__force s3c_gpio_pull_t)0x00)
#define S3C_GPIO_PULL_DOWN	((__force s3c_gpio_pull_t)0x01)
#define S3C_GPIO_PULL_UP	((__force s3c_gpio_pull_t)0x02)

extern int s3c_gpio_setpull(unsigned int pin, s3c_gpio_pull_t pull);

extern s3c_gpio_pull_t s3c_gpio_getpull(unsigned int pin);

#define S5P_GPIO_DRVSTR_LV1	((__force s5p_gpio_drvstr_t)0x00)
#define S5P_GPIO_DRVSTR_LV2	((__force s5p_gpio_drvstr_t)0x01)
#define S5P_GPIO_DRVSTR_LV3	((__force s5p_gpio_drvstr_t)0x10)
#define S5P_GPIO_DRVSTR_LV4	((__force s5p_gpio_drvstr_t)0x11)

extern s5p_gpio_drvstr_t s5p_gpio_get_drvstr(unsigned int pin);

extern int s5p_gpio_set_drvstr(unsigned int pin, s5p_gpio_drvstr_t drvstr);

#endif /* __PLAT_GPIO_CFG_H */
