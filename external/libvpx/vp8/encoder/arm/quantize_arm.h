


#ifndef QUANTIZE_ARM_H
#define QUANTIZE_ARM_H

#if HAVE_ARMV7
extern prototype_quantize_block(vp8_fast_quantize_b_neon);

//#undef  vp8_quantize_fastquantb
//#define vp8_quantize_fastquantb vp8_fast_quantize_b_neon

#endif

#endif
