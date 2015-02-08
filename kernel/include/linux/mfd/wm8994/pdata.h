

#ifndef __MFD_WM8994_PDATA_H__
#define __MFD_WM8994_PDATA_H__

#define WM8994_NUM_LDO   2
#define WM8994_NUM_GPIO 11

struct wm8994_ldo_pdata {
	/** GPIOs to enable regulator, 0 or less if not available */
	int enable;

	const char *supply;
	struct regulator_init_data *init_data;
};

#define WM8994_CONFIGURE_GPIO 0x8000

#define WM8994_DRC_REGS 5
#define WM8994_EQ_REGS  19

struct wm8994_drc_cfg {
        const char *name;
        u16 regs[WM8994_DRC_REGS];
};

struct wm8994_retune_mobile_cfg {
        const char *name;
        unsigned int rate;
        u16 regs[WM8994_EQ_REGS];
};

struct wm8994_pdata {
	int gpio_base;

	/**
	 * Default values for GPIOs if non-zero, WM8994_CONFIGURE_GPIO
	 * can be used for all zero values.
	 */
	int gpio_defaults[WM8994_NUM_GPIO];

	struct wm8994_ldo_pdata ldo[WM8994_NUM_LDO];

	int irq_base;  /** Base IRQ number for WM8994, required for IRQs */

        int num_drc_cfgs;
        struct wm8994_drc_cfg *drc_cfgs;

        int num_retune_mobile_cfgs;
        struct wm8994_retune_mobile_cfg *retune_mobile_cfgs;

        /* LINEOUT can be differential or single ended */
        unsigned int lineout1_diff:1;
        unsigned int lineout2_diff:1;

        /* Common mode feedback */
        unsigned int lineout1fb:1;
        unsigned int lineout2fb:1;

        /* Microphone biases: 0=0.9*AVDD1 1=0.65*AVVD1 */
        unsigned int micbias1_lvl:1;
        unsigned int micbias2_lvl:1;

        /* Jack detect threashold levels, see datasheet for values */
        unsigned int jd_scthr:2;
        unsigned int jd_thr:2;
};

#endif
