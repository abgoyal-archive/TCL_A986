

enum s3c2410_dma_buffresult {
	S3C2410_RES_OK,
	S3C2410_RES_ERR,
	S3C2410_RES_ABORT
};

enum s3c2410_dmasrc {
	S3C2410_DMASRC_HW,		/* source is memory */
	S3C2410_DMASRC_MEM		/* source is hardware */
};


enum s3c2410_chan_op {
	S3C2410_DMAOP_START,
	S3C2410_DMAOP_STOP,
	S3C2410_DMAOP_PAUSE,
	S3C2410_DMAOP_RESUME,
	S3C2410_DMAOP_FLUSH,
	S3C2410_DMAOP_TIMEOUT,		/* internal signal to handler */
	S3C2410_DMAOP_STARTED,		/* indicate channel started */
};

struct s3c2410_dma_client {
	char                *name;
};

struct s3c2410_dma_chan;


typedef void (*s3c2410_dma_cbfn_t)(struct s3c2410_dma_chan *,
				   void *buf, int size,
				   enum s3c2410_dma_buffresult result);

typedef int  (*s3c2410_dma_opfn_t)(struct s3c2410_dma_chan *,
				   enum s3c2410_chan_op );




extern int s3c2410_dma_request(unsigned int channel,
			       struct s3c2410_dma_client *, void *dev);



extern int s3c2410_dma_ctrl(unsigned int channel, enum s3c2410_chan_op op);


extern int s3c2410_dma_setflags(unsigned int channel,
				unsigned int flags);


extern int s3c2410_dma_free(unsigned int channel, struct s3c2410_dma_client *);


extern int s3c2410_dma_enqueue(unsigned int channel, void *id,
			       dma_addr_t data, int size);


extern int s3c2410_dma_config(unsigned int channel, int xferunit);


extern int s3c2410_dma_devconfig(unsigned int channel,
		enum s3c2410_dmasrc source, unsigned long devaddr);


extern int s3c2410_dma_getposition(unsigned int channel,
				   dma_addr_t *src, dma_addr_t *dest);

extern int s3c2410_dma_set_opfn(unsigned int, s3c2410_dma_opfn_t rtn);
extern int s3c2410_dma_set_buffdone_fn(unsigned int, s3c2410_dma_cbfn_t rtn);


