


#include "vpx_ports/config.h"
#include "recon.h"
#include "blockd.h"

extern void vp8_recon16x16mb_neon(unsigned char *pred_ptr, short *diff_ptr, unsigned char *dst_ptr, int ystride, unsigned char *udst_ptr, unsigned char *vdst_ptr);

void vp8_recon_mb_neon(const vp8_recon_rtcd_vtable_t *rtcd, MACROBLOCKD *x)
{
    unsigned char *pred_ptr = &x->predictor[0];
    short *diff_ptr = &x->diff[0];
    unsigned char *dst_ptr = x->dst.y_buffer;
    unsigned char *udst_ptr = x->dst.u_buffer;
    unsigned char *vdst_ptr = x->dst.v_buffer;
    int ystride = x->dst.y_stride;
    /*int uv_stride = x->dst.uv_stride;*/

    vp8_recon16x16mb_neon(pred_ptr, diff_ptr, dst_ptr, ystride, udst_ptr, vdst_ptr);
}
