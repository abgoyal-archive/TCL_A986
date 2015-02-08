


#ifndef RECON_X86_H
#define RECON_X86_H


#if HAVE_MMX
extern prototype_recon_block(vp8_recon_b_mmx);
extern prototype_copy_block(vp8_copy_mem8x8_mmx);
extern prototype_copy_block(vp8_copy_mem8x4_mmx);
extern prototype_copy_block(vp8_copy_mem16x16_mmx);


#if !CONFIG_RUNTIME_CPU_DETECT
#undef  vp8_recon_recon
#define vp8_recon_recon vp8_recon_b_mmx

#undef  vp8_recon_copy8x8
#define vp8_recon_copy8x8 vp8_copy_mem8x8_mmx

#undef  vp8_recon_copy8x4
#define vp8_recon_copy8x4 vp8_copy_mem8x4_mmx

#undef  vp8_recon_copy16x16
#define vp8_recon_copy16x16 vp8_copy_mem16x16_mmx

#endif
#endif

#if HAVE_SSE2
extern prototype_recon_block(vp8_recon2b_sse2);
extern prototype_recon_block(vp8_recon4b_sse2);
extern prototype_copy_block(vp8_copy_mem16x16_sse2);

#if !CONFIG_RUNTIME_CPU_DETECT
#undef  vp8_recon_recon2
#define vp8_recon_recon2 vp8_recon2b_sse2

#undef  vp8_recon_recon4
#define vp8_recon_recon4 vp8_recon4b_sse2

#undef  vp8_recon_copy16x16
#define vp8_recon_copy16x16 vp8_copy_mem16x16_sse2

#endif
#endif
#endif
