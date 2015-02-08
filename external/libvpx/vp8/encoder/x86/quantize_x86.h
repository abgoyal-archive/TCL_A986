

#ifndef QUANTIZE_X86_H
#define QUANTIZE_X86_H


#if HAVE_MMX

#endif


#if HAVE_SSE2
extern prototype_quantize_block(vp8_regular_quantize_b_sse2);

#if !CONFIG_RUNTIME_CPU_DETECT


#endif

#endif


#endif
