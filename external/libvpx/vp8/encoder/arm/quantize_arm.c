


#include <math.h>
#include "vpx_mem/vpx_mem.h"

#include "quantize.h"
#include "entropy.h"
#include "predictdc.h"

DECLARE_ALIGNED(16, const short, vp8_rvsplus1_default_zig_zag1d[16]) =
{
    1,  2,  6,  7,
    3,  5,  8,  13,
    4,  9,  12, 14,
    10, 11, 15, 16,
};


extern int vp8_fast_quantize_b_neon_func(short *coeff_ptr, short *zbin_ptr, short *qcoeff_ptr, short *dqcoeff_ptr, short *dequant_ptr, const short *scan_mask, short *round_ptr, short *quant_ptr);

void vp8_fast_quantize_b_neon(BLOCK *b, BLOCKD *d)
{
    d->eob = vp8_fast_quantize_b_neon_func(b->coeff, b->zbin, d->qcoeff, d->dqcoeff, d->dequant, vp8_rvsplus1_default_zig_zag1d, b->round, b->quant);
}

