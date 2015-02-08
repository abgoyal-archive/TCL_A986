
#ifndef __LINUX_SERIAL_SCI_H
#define __LINUX_SERIAL_SCI_H

#include <linux/serial_core.h>
#include <linux/sh_dma.h>


/* Offsets into the sci_port->irqs array */
enum {
	SCIx_ERI_IRQ,
	SCIx_RXI_IRQ,
	SCIx_TXI_IRQ,
	SCIx_BRI_IRQ,
	SCIx_NR_IRQS,
};

struct device;

struct plat_sci_port {
	void __iomem	*membase;		/* io cookie */
	unsigned long	mapbase;		/* resource base */
	unsigned int	irqs[SCIx_NR_IRQS];	/* ERI, RXI, TXI, BRI */
	unsigned int	type;			/* SCI / SCIF / IRDA */
	upf_t		flags;			/* UPF_* flags */
	char		*clk;			/* clock string */
	struct device	*dma_dev;
#ifdef CONFIG_SERIAL_SH_SCI_DMA
	unsigned int dma_slave_tx;
	unsigned int dma_slave_rx;
#endif
};

#endif /* __LINUX_SERIAL_SCI_H */
