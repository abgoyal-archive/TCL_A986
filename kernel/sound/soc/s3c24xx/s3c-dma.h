

#ifndef _S3C_AUDIO_H
#define _S3C_AUDIO_H

#define ST_RUNNING		(1<<0)
#define ST_OPENED		(1<<1)

struct s3c_dma_params {
	struct s3c2410_dma_client *client;	/* stream identifier */
	int channel;				/* Channel ID */
	dma_addr_t dma_addr;
	int dma_size;			/* Size of the DMA transfer */
};

#define S3C24XX_DAI_I2S			0

/* platform data */
extern struct snd_soc_platform s3c24xx_soc_platform;
extern struct snd_ac97_bus_ops s3c24xx_ac97_ops;

#endif
