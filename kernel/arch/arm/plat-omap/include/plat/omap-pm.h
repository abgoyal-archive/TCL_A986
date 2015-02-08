

#ifndef ASM_ARM_ARCH_OMAP_OMAP_PM_H
#define ASM_ARM_ARCH_OMAP_OMAP_PM_H

#include <linux/device.h>
#include <linux/cpufreq.h>

#include "powerdomain.h"

struct omap_opp {
	unsigned long rate;
	u8 opp_id;
	u16 min_vdd;
};

extern struct omap_opp *mpu_opps;
extern struct omap_opp *dsp_opps;
extern struct omap_opp *l3_opps;

#define OCP_TARGET_AGENT		1
#define OCP_INITIATOR_AGENT		2

int __init omap_pm_if_early_init(struct omap_opp *mpu_opp_table,
				 struct omap_opp *dsp_opp_table,
				 struct omap_opp *l3_opp_table);

int __init omap_pm_if_init(void);

void omap_pm_if_exit(void);



void omap_pm_set_max_mpu_wakeup_lat(struct device *dev, long t);


void omap_pm_set_min_bus_tput(struct device *dev, u8 agent_id, unsigned long r);


void omap_pm_set_max_dev_wakeup_lat(struct device *dev, long t);


void omap_pm_set_max_sdma_lat(struct device *dev, long t);



const struct omap_opp *omap_pm_dsp_get_opp_table(void);

void omap_pm_dsp_set_min_opp(u8 opp_id);

u8 omap_pm_dsp_get_opp(void);



struct cpufreq_frequency_table **omap_pm_cpu_get_freq_table(void);

void omap_pm_cpu_set_freq(unsigned long f);

unsigned long omap_pm_cpu_get_freq(void);



int omap_pm_get_dev_context_loss_count(struct device *dev);


#endif
