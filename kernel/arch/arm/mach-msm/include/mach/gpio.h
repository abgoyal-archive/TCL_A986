
#ifndef __ASM_ARCH_MSM_GPIO_H
#define __ASM_ARCH_MSM_GPIO_H

struct msm_gpio {
	u32 gpio_cfg;
	const char *label;
};

int msm_gpios_request_enable(const struct msm_gpio *table, int size);

void msm_gpios_disable_free(const struct msm_gpio *table, int size);

int msm_gpios_request(const struct msm_gpio *table, int size);

void msm_gpios_free(const struct msm_gpio *table, int size);

int msm_gpios_enable(const struct msm_gpio *table, int size);

void msm_gpios_disable(const struct msm_gpio *table, int size);

/* GPIO TLMM (Top Level Multiplexing) Definitions */

/* GPIO TLMM: Function -- GPIO specific */

/* GPIO TLMM: Direction */
enum {
	GPIO_INPUT,
	GPIO_OUTPUT,
};

/* GPIO TLMM: Pullup/Pulldown */
enum {
	GPIO_NO_PULL,
	GPIO_PULL_DOWN,
	GPIO_KEEPER,
	GPIO_PULL_UP,
};

/* GPIO TLMM: Drive Strength */
enum {
	GPIO_2MA,
	GPIO_4MA,
	GPIO_6MA,
	GPIO_8MA,
	GPIO_10MA,
	GPIO_12MA,
	GPIO_14MA,
	GPIO_16MA,
};

enum {
	GPIO_ENABLE,
	GPIO_DISABLE,
};

#define GPIO_CFG(gpio, func, dir, pull, drvstr) \
	((((gpio) & 0x3FF) << 4)        |	  \
	 ((func) & 0xf)                  |	  \
	 (((dir) & 0x1) << 14)           |	  \
	 (((pull) & 0x3) << 15)          |	  \
	 (((drvstr) & 0xF) << 17))

#define GPIO_PIN(gpio_cfg)    (((gpio_cfg) >>  4) & 0x3ff)
#define GPIO_FUNC(gpio_cfg)   (((gpio_cfg) >>  0) & 0xf)
#define GPIO_DIR(gpio_cfg)    (((gpio_cfg) >> 14) & 0x1)
#define GPIO_PULL(gpio_cfg)   (((gpio_cfg) >> 15) & 0x3)
#define GPIO_DRVSTR(gpio_cfg) (((gpio_cfg) >> 17) & 0xf)

int gpio_tlmm_config(unsigned config, unsigned disable);

#endif /* __ASM_ARCH_MSM_GPIO_H */
