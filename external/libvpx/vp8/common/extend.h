


#ifndef __INC_EXTEND_H
#define __INC_EXTEND_H

#include "vpx_scale/yv12config.h"

void Extend(YV12_BUFFER_CONFIG *ybf);
void vp8_extend_mb_row(YV12_BUFFER_CONFIG *ybf, unsigned char *YPtr, unsigned char *UPtr, unsigned char *VPtr);
void vp8_extend_to_multiple_of16(YV12_BUFFER_CONFIG *ybf, int width, int height);

#endif
