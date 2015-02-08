


#include "encodemb.h"
#include "reconinter.h"
#include "quantize.h"
#include "invtrans.h"
#include "recon.h"
#include "reconintra.h"
#include "dct.h"
#include "vpx_mem/vpx_mem.h"

extern void vp8_subtract_b_neon_func(short *diff, unsigned char *src, unsigned char *pred, int stride, int pitch);

void vp8_subtract_b_neon(BLOCK *be, BLOCKD *bd, int pitch)
{
    unsigned char *src_ptr = (*(be->base_src) + be->src);
    short *diff_ptr = be->src_diff;
    unsigned char *pred_ptr = bd->predictor;
    int src_stride = be->src_stride;

    vp8_subtract_b_neon_func(diff_ptr, src_ptr, pred_ptr, src_stride, pitch);
}
