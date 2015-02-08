


#ifndef __INC_PICKINTER_H
#define __INC_PICKINTER_H
#include "vpx_ports/config.h"
#include "onyxc_int.h"

#define RD_ESTIMATE(RM,DM,R,D) ( ((128+(R)*(RM)) >> 8) + (DM)*(D) )
extern int vp8_pick_intra4x4mby_modes(const VP8_ENCODER_RTCD *, MACROBLOCK *mb, int *Rate, int *Distortion);
extern int vp8_pick_intra_mbuv_mode(MACROBLOCK *mb);
extern int vp8_pick_inter_mode(VP8_COMP *cpi, MACROBLOCK *x, int recon_yoffset, int recon_uvoffset, int *returnrate, int *returndistortion, int *returnintra);
#endif
