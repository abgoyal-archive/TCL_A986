


#ifndef DETOKENIZE_ARM_H
#define DETOKENIZE_ARM_H

#if HAVE_ARMV6
#if CONFIG_ARM_ASM_DETOK
void vp8_init_detokenizer(VP8D_COMP *dx);
void vp8_decode_mb_tokens_v6(DETOK *detoken, int type);
#endif
#endif

#endif
