


#ifndef VP8E_H
#define VP8E_H
#include "vpx/vpx_codec_impl_top.h"

#if defined(VPX_CODEC_DISABLE_COMPAT) && VPX_CODEC_DISABLE_COMPAT
#error "Backwards compatibility disabled: don't include vp8e.h"
#endif

#include "vp8cx.h"
DECLSPEC_DEPRECATED extern vpx_codec_iface_t vpx_enc_vp8_algo DEPRECATED;


enum
{
    VP8E_SET_REFERENCE     = VP8_SET_REFERENCE,
    VP8E_COPY_REFERENCE    = VP8_COPY_REFERENCE,
    VP8E_SET_PREVIEWPP     = VP8_SET_POSTPROC,
    VP8E_SET_FLUSHFLAG     = 4,
    VP8E_SET_FRAMETYPE     = 10,
    VP8E_SET_ENCODING_MODE = 12
};

#define NORMAL_FRAME   (0)
#define KEY_FRAME      (1)

VPX_CTRL_USE_TYPE_DEPRECATED(VP8E_SET_REFERENCE,   vpx_ref_frame_t *)
VPX_CTRL_USE_TYPE_DEPRECATED(VP8E_COPY_REFERENCE,  vpx_ref_frame_t *)
VPX_CTRL_USE_TYPE_DEPRECATED(VP8E_SET_PREVIEWPP,   vp8_postproc_cfg_t *)


/* Flush is done by calling vpx_codec_encode with a NULL input image. */
VPX_CTRL_USE_TYPE_DEPRECATED(VP8E_SET_FLUSHFLAG,          int)


VPX_CTRL_USE_TYPE_DEPRECATED(VP8E_SET_FRAMETYPE,          int)


VPX_CTRL_USE_TYPE_DEPRECATED(VP8E_SET_ENCODING_MODE, vp8e_encoding_mode)
#include "vpx/vpx_codec_impl_bottom.h"
#endif
