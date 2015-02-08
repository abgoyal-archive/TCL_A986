


#ifndef DEQUANTIZE_H
#define DEQUANTIZE_H
#include "blockd.h"

#define prototype_dequant_block(sym) \
    void sym(BLOCKD *x)

#define prototype_dequant_idct_add(sym) \
    void sym(short *input, short *dq, \
             unsigned char *pred, unsigned char *output, \
             int pitch, int stride)

#define prototype_dequant_dc_idct_add(sym) \
    void sym(short *input, short *dq, \
             unsigned char *pred, unsigned char *output, \
             int pitch, int stride, \
             int dc)

#define prototype_dequant_dc_idct_add_y_block(sym) \
    void sym(short *q, short *dq, \
             unsigned char *pre, unsigned char *dst, \
             int stride, char *eobs, short *dc)

#define prototype_dequant_idct_add_y_block(sym) \
    void sym(short *q, short *dq, \
             unsigned char *pre, unsigned char *dst, \
             int stride, char *eobs)

#define prototype_dequant_idct_add_uv_block(sym) \
    void sym(short *q, short *dq, \
             unsigned char *pre, unsigned char *dst_u, \
             unsigned char *dst_v, int stride, char *eobs)

#if ARCH_X86 || ARCH_X86_64
#include "x86/dequantize_x86.h"
#endif

#if ARCH_ARM
#include "arm/dequantize_arm.h"
#endif

#ifndef vp8_dequant_block
#define vp8_dequant_block vp8_dequantize_b_c
#endif
extern prototype_dequant_block(vp8_dequant_block);

#ifndef vp8_dequant_idct_add
#define vp8_dequant_idct_add vp8_dequant_idct_add_c
#endif
extern prototype_dequant_idct_add(vp8_dequant_idct_add);

#ifndef vp8_dequant_dc_idct_add
#define vp8_dequant_dc_idct_add vp8_dequant_dc_idct_add_c
#endif
extern prototype_dequant_dc_idct_add(vp8_dequant_dc_idct_add);

#ifndef vp8_dequant_dc_idct_add_y_block
#define vp8_dequant_dc_idct_add_y_block vp8_dequant_dc_idct_add_y_block_c
#endif
extern prototype_dequant_dc_idct_add_y_block(vp8_dequant_dc_idct_add_y_block);

#ifndef vp8_dequant_idct_add_y_block
#define vp8_dequant_idct_add_y_block vp8_dequant_idct_add_y_block_c
#endif
extern prototype_dequant_idct_add_y_block(vp8_dequant_idct_add_y_block);

#ifndef vp8_dequant_idct_add_uv_block
#define vp8_dequant_idct_add_uv_block vp8_dequant_idct_add_uv_block_c
#endif
extern prototype_dequant_idct_add_uv_block(vp8_dequant_idct_add_uv_block);


typedef prototype_dequant_block((*vp8_dequant_block_fn_t));

typedef prototype_dequant_idct_add((*vp8_dequant_idct_add_fn_t));

typedef prototype_dequant_dc_idct_add((*vp8_dequant_dc_idct_add_fn_t));

typedef prototype_dequant_dc_idct_add_y_block((*vp8_dequant_dc_idct_add_y_block_fn_t));

typedef prototype_dequant_idct_add_y_block((*vp8_dequant_idct_add_y_block_fn_t));

typedef prototype_dequant_idct_add_uv_block((*vp8_dequant_idct_add_uv_block_fn_t));

typedef struct
{
    vp8_dequant_block_fn_t               block;
    vp8_dequant_idct_add_fn_t            idct_add;
    vp8_dequant_dc_idct_add_fn_t         dc_idct_add;
    vp8_dequant_dc_idct_add_y_block_fn_t dc_idct_add_y_block;
    vp8_dequant_idct_add_y_block_fn_t    idct_add_y_block;
    vp8_dequant_idct_add_uv_block_fn_t   idct_add_uv_block;
} vp8_dequant_rtcd_vtable_t;

#if CONFIG_RUNTIME_CPU_DETECT
#define DEQUANT_INVOKE(ctx,fn) (ctx)->fn
#else
#define DEQUANT_INVOKE(ctx,fn) vp8_dequant_##fn
#endif

#endif
