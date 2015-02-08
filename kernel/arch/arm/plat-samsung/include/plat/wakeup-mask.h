

#ifndef __PLAT_WAKEUP_MASK_H
#define __PLAT_WAKEUP_MASK_H __file__

/* if no irq yet defined, but still want to mask */
#define NO_WAKEUP_IRQ (0x90000000)

struct samsung_wakeup_mask {
	unsigned int	irq;
	u32		bit;
};

extern void samsung_sync_wakemask(void __iomem *reg,
				  struct samsung_wakeup_mask *masks,
				  int nr_masks);

#endif /* __PLAT_WAKEUP_MASK_H */
