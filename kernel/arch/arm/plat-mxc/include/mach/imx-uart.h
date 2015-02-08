

#ifndef ASMARM_ARCH_UART_H
#define ASMARM_ARCH_UART_H

#define IMXUART_HAVE_RTSCTS (1<<0)
#define IMXUART_IRDA        (1<<1)

struct imxuart_platform_data {
	int (*init)(struct platform_device *pdev);
	void (*exit)(struct platform_device *pdev);
	unsigned int flags;
	void (*irda_enable)(int enable);
	unsigned int irda_inv_rx:1;
	unsigned int irda_inv_tx:1;
	unsigned short transceiver_delay;
};

#endif
