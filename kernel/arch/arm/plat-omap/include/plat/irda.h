
#ifndef ASMARM_ARCH_IRDA_H
#define ASMARM_ARCH_IRDA_H

/* board specific transceiver capabilities */

#define IR_SEL		1	/* Selects IrDA */
#define IR_SIRMODE	2
#define IR_FIRMODE	4
#define IR_MIRMODE	8

struct omap_irda_config {
	int transceiver_cap;
	int (*transceiver_mode)(struct device *dev, int mode);
	int (*select_irda)(struct device *dev, int state);
	int rx_channel;
	int tx_channel;
	unsigned long dest_start;
	unsigned long src_start;
	int tx_trigger;
	int rx_trigger;
	int mode;
};

#endif
