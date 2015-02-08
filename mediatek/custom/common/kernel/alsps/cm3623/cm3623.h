
#ifndef __CM3623_H__
#define __CM3623_H__

#include <linux/ioctl.h>

/*ALS Command*/
#define SD_ALS      (1      << 0)
#define WDM         (1      << 1)
#define IT_ALS      (0x03   << 2)
#define THD_ALS     (0x03   << 4)
#define GAIN_ALS    (0x03   << 6)

/*Proximity sensor command*/
#define SD_PS       (1      << 0)
#define INT_PS      (1      << 2)
#define INT_ALS     (1      << 3)
#define IT_PS       (0x03   << 4)
#define DR_PS       (0x03   << 6)

#endif

