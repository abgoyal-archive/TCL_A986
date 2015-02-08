

#ifndef _ATMEL_PCM_H
#define _ATMEL_PCM_H

#include <linux/atmel-ssc.h>

struct atmel_pdc_regs {
	unsigned int	xpr;		/* PDC recv/trans pointer */
	unsigned int	xcr;		/* PDC recv/trans counter */
	unsigned int	xnpr;		/* PDC next recv/trans pointer */
	unsigned int	xncr;		/* PDC next recv/trans counter */
	unsigned int	ptcr;		/* PDC transfer control */
};

struct atmel_ssc_mask {
	u32	ssc_enable;		/* SSC recv/trans enable */
	u32	ssc_disable;		/* SSC recv/trans disable */
	u32	ssc_endx;		/* SSC ENDTX or ENDRX */
	u32	ssc_endbuf;		/* SSC TXBUFE or RXBUFF */
	u32	pdc_enable;		/* PDC recv/trans enable */
	u32	pdc_disable;		/* PDC recv/trans disable */
};

struct atmel_pcm_dma_params {
	char *name;			/* stream identifier */
	int pdc_xfer_size;		/* PDC counter increment in bytes */
	struct ssc_device *ssc;		/* SSC device for stream */
	struct atmel_pdc_regs *pdc;	/* PDC receive or transmit registers */
	struct atmel_ssc_mask *mask;	/* SSC & PDC status bits */
	struct snd_pcm_substream *substream;
	void (*dma_intr_handler)(u32, struct snd_pcm_substream *);
};

extern struct snd_soc_platform atmel_soc_platform;


#define ssc_readx(base, reg)            (__raw_readl((base) + (reg)))
#define ssc_writex(base, reg, value)    __raw_writel((value), (base) + (reg))

#endif /* _ATMEL_PCM_H */
