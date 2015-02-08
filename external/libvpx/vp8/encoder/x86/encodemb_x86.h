


#ifndef ENCODEMB_X86_H
#define ENCODEMB_X86_H


#if HAVE_MMX
extern prototype_berr(vp8_block_error_mmx);
extern prototype_mberr(vp8_mbblock_error_mmx);
extern prototype_mbuverr(vp8_mbuverror_mmx);
extern prototype_subb(vp8_subtract_b_mmx);
extern prototype_submby(vp8_subtract_mby_mmx);
extern prototype_submbuv(vp8_subtract_mbuv_mmx);


#if !CONFIG_RUNTIME_CPU_DETECT
#undef  vp8_encodemb_berr
#define vp8_encodemb_berr vp8_block_error_mmx

#undef  vp8_encodemb_mberr
#define vp8_encodemb_mberr vp8_mbblock_error_mmx

#undef  vp8_encodemb_mbuverr
#define vp8_encodemb_mbuverr vp8_mbuverror_mmx

#undef  vp8_encodemb_subb
#define vp8_encodemb_subb vp8_subtract_b_mmx

#undef  vp8_encodemb_submby
#define vp8_encodemb_submby vp8_subtract_mby_mmx

#undef  vp8_encodemb_submbuv
#define vp8_encodemb_submbuv vp8_subtract_mbuv_mmx

#endif
#endif


#if HAVE_SSE2
extern prototype_berr(vp8_block_error_xmm);
extern prototype_mberr(vp8_mbblock_error_xmm);
extern prototype_mbuverr(vp8_mbuverror_xmm);
extern prototype_subb(vp8_subtract_b_sse2);
extern prototype_submby(vp8_subtract_mby_sse2);
extern prototype_submbuv(vp8_subtract_mbuv_sse2);

#if !CONFIG_RUNTIME_CPU_DETECT
#undef  vp8_encodemb_berr
#define vp8_encodemb_berr vp8_block_error_xmm

#undef  vp8_encodemb_mberr
#define vp8_encodemb_mberr vp8_mbblock_error_xmm

#undef  vp8_encodemb_mbuverr
#define vp8_encodemb_mbuverr vp8_mbuverror_xmm

#undef  vp8_encodemb_subb
#define vp8_encodemb_subb vp8_subtract_b_sse2

#undef  vp8_encodemb_submby
#define vp8_encodemb_submby vp8_subtract_mby_sse2

#undef  vp8_encodemb_submbuv
#define vp8_encodemb_submbuv vp8_subtract_mbuv_sse2

#endif
#endif


#endif
