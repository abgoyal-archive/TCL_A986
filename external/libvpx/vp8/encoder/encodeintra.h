


#ifndef _ENCODEINTRA_H_
#define _ENCODEINTRA_H_
#include "onyx_int.h"

void vp8_encode_intra16x16mby(const VP8_ENCODER_RTCD *, MACROBLOCK *x);
void vp8_encode_intra16x16mbuv(const VP8_ENCODER_RTCD *, MACROBLOCK *x);
void vp8_encode_intra4x4mby(const VP8_ENCODER_RTCD *, MACROBLOCK *mb);
void vp8_encode_intra4x4block(const VP8_ENCODER_RTCD *, MACROBLOCK *x, BLOCK *be, BLOCKD *b, int best_mode);
void vp8_update_mode_context(int *abmode, int *lbmode, int i, int best_mode);
void vp8_encode_intra4x4block_rd(const VP8_ENCODER_RTCD *, MACROBLOCK *x, BLOCK *be, BLOCKD *b, int best_mode);
void vp8_encode_intra16x16mbyrd(const VP8_ENCODER_RTCD *, MACROBLOCK *x);
void vp8_encode_intra16x16mbuvrd(const VP8_ENCODER_RTCD *, MACROBLOCK *x);

#endif
