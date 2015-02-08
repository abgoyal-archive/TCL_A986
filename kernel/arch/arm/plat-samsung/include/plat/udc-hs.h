

enum s3c_hsotg_dmamode {
	S3C_HSOTG_DMA_NONE,	/* do not use DMA at-all */
	S3C_HSOTG_DMA_ONLY,	/* always use DMA */
	S3C_HSOTG_DMA_DRV,	/* DMA is chosen by driver */
};

struct s3c_hsotg_plat {
	enum s3c_hsotg_dmamode	dma;
	unsigned int		is_osc : 1;
};
