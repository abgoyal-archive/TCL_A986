

#ifndef __PLAT_SHIRQ_H
#define __PLAT_SHIRQ_H

#include <linux/irq.h>
#include <linux/types.h>

struct shirq_dev_config {
	u32 virq;
	u32 enb_mask;
	u32 status_mask;
	u32 clear_mask;
};

struct shirq_regs {
	void __iomem *base;
	u32 enb_reg;
	u32 reset_to_enb;
	u32 status_reg;
	u32 status_reg_mask;
	u32 clear_reg;
	u32 reset_to_clear;
};

struct spear_shirq {
	u32 irq;
	struct shirq_dev_config *dev_config;
	u32 dev_count;
	struct shirq_regs regs;
};

int spear_shirq_register(struct spear_shirq *shirq);

#endif /* __PLAT_SHIRQ_H */
