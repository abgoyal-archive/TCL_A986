

#ifndef __S3C_PL330_PDATA_H
#define __S3C_PL330_PDATA_H

#include <plat/s3c-dma-pl330.h>

struct s3c_pl330_platdata {
	enum dma_ch peri[32];
};

#endif /* __S3C_PL330_PDATA_H */
