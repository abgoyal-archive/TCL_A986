


#include "vp8.h"

#ifndef VP8DX_H
#define VP8DX_H
#include "vpx/vpx_codec_impl_top.h"

extern vpx_codec_iface_t  vpx_codec_vp8_dx_algo;
extern vpx_codec_iface_t* vpx_codec_vp8_dx(void);

/* Include controls common to both the encoder and decoder */
#include "vp8.h"


/*! @} - end defgroup vp8_decoder */


#include "vpx/vpx_codec_impl_bottom.h"
#endif
