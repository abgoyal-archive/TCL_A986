


#ifndef __INC_INVTRANS_H
#define __INC_INVTRANS_H

#include "vpx_ports/config.h"
#include "idct.h"
#include "blockd.h"
extern void vp8_inverse_transform_b(const vp8_idct_rtcd_vtable_t *rtcd, BLOCKD *b, int pitch);
extern void vp8_inverse_transform_mb(const vp8_idct_rtcd_vtable_t *rtcd, MACROBLOCKD *x);
extern void vp8_inverse_transform_mby(const vp8_idct_rtcd_vtable_t *rtcd, MACROBLOCKD *x);
extern void vp8_inverse_transform_mbuv(const vp8_idct_rtcd_vtable_t *rtcd, MACROBLOCKD *x);

#endif
