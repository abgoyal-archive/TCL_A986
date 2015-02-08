


#ifndef POSTPROC_X86_H
#define POSTPROC_X86_H


#if HAVE_MMX
extern prototype_postproc_inplace(vp8_mbpost_proc_down_mmx);
extern prototype_postproc(vp8_post_proc_down_and_across_mmx);
extern prototype_postproc_addnoise(vp8_plane_add_noise_mmx);

#if !CONFIG_RUNTIME_CPU_DETECT
#undef  vp8_postproc_down
#define vp8_postproc_down vp8_mbpost_proc_down_mmx

#undef  vp8_postproc_downacross
#define vp8_postproc_downacross vp8_post_proc_down_and_across_mmx

#undef  vp8_postproc_addnoise
#define vp8_postproc_addnoise vp8_plane_add_noise_mmx

#endif
#endif


#if HAVE_SSE2
extern prototype_postproc_inplace(vp8_mbpost_proc_down_xmm);
extern prototype_postproc_inplace(vp8_mbpost_proc_across_ip_xmm);
extern prototype_postproc(vp8_post_proc_down_and_across_xmm);
extern prototype_postproc_addnoise(vp8_plane_add_noise_wmt);

#if !CONFIG_RUNTIME_CPU_DETECT
#undef  vp8_postproc_down
#define vp8_postproc_down vp8_mbpost_proc_down_xmm

#undef  vp8_postproc_across
#define vp8_postproc_across vp8_mbpost_proc_across_ip_xmm

#undef  vp8_postproc_downacross
#define vp8_postproc_downacross vp8_post_proc_down_and_across_xmm

#undef  vp8_postproc_addnoise
#define vp8_postproc_addnoise vp8_plane_add_noise_wmt


#endif
#endif

#endif
