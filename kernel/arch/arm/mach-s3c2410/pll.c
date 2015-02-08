

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <plat/cpu.h>
#include <plat/cpu-freq-core.h>

static struct cpufreq_frequency_table pll_vals_12MHz[] = {
    { .frequency = 34000000,  .index = PLLVAL(82, 2, 3),   },
    { .frequency = 45000000,  .index = PLLVAL(82, 1, 3),   },
    { .frequency = 51000000,  .index = PLLVAL(161, 3, 3),  },
    { .frequency = 48000000,  .index = PLLVAL(120, 2, 3),  },
    { .frequency = 56000000,  .index = PLLVAL(142, 2, 3),  },
    { .frequency = 68000000,  .index = PLLVAL(82, 2, 2),   },
    { .frequency = 79000000,  .index = PLLVAL(71, 1, 2),   },
    { .frequency = 85000000,  .index = PLLVAL(105, 2, 2),  },
    { .frequency = 90000000,  .index = PLLVAL(112, 2, 2),  },
    { .frequency = 101000000, .index = PLLVAL(127, 2, 2),  },
    { .frequency = 113000000, .index = PLLVAL(105, 1, 2),  },
    { .frequency = 118000000, .index = PLLVAL(150, 2, 2),  },
    { .frequency = 124000000, .index = PLLVAL(116, 1, 2),  },
    { .frequency = 135000000, .index = PLLVAL(82, 2, 1),   },
    { .frequency = 147000000, .index = PLLVAL(90, 2, 1),   },
    { .frequency = 152000000, .index = PLLVAL(68, 1, 1),   },
    { .frequency = 158000000, .index = PLLVAL(71, 1, 1),   },
    { .frequency = 170000000, .index = PLLVAL(77, 1, 1),   },
    { .frequency = 180000000, .index = PLLVAL(82, 1, 1),   },
    { .frequency = 186000000, .index = PLLVAL(85, 1, 1),   },
    { .frequency = 192000000, .index = PLLVAL(88, 1, 1),   },
    { .frequency = 203000000, .index = PLLVAL(161, 3, 1),  },

    /* 2410A extras */

    { .frequency = 210000000, .index = PLLVAL(132, 2, 1),  },
    { .frequency = 226000000, .index = PLLVAL(105, 1, 1),  },
    { .frequency = 266000000, .index = PLLVAL(125, 1, 1),  },
    { .frequency = 268000000, .index = PLLVAL(126, 1, 1),  },
    { .frequency = 270000000, .index = PLLVAL(127, 1, 1),  },
};

static int s3c2410_plls_add(struct sys_device *dev)
{
	return s3c_plltab_register(pll_vals_12MHz, ARRAY_SIZE(pll_vals_12MHz));
}

static struct sysdev_driver s3c2410_plls_drv = {
	.add	= s3c2410_plls_add,
};

static int __init s3c2410_pll_init(void)
{
	return sysdev_driver_register(&s3c2410_sysclass, &s3c2410_plls_drv);

}

arch_initcall(s3c2410_pll_init);

static struct sysdev_driver s3c2410a_plls_drv = {
	.add	= s3c2410_plls_add,
};

static int __init s3c2410a_pll_init(void)
{
	return sysdev_driver_register(&s3c2410a_sysclass, &s3c2410a_plls_drv);
}

arch_initcall(s3c2410a_pll_init);
