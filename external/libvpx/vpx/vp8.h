


#ifndef VP8_H
#define VP8_H
#include "vpx/vpx_codec_impl_top.h"

enum vp8_dec_control_id
{
    VP8_SET_REFERENCE       = 1,    /**< pass in an external frame into decoder to be used as reference frame */
    VP8_COPY_REFERENCE      = 2,    /**< get a copy of reference frame from the decoder */
    VP8_SET_POSTPROC        = 3,    /**< set decoder's the post processing settings  */
    VP8_COMMON_CTRL_ID_MAX
};

enum vp8_postproc_level
{
    VP8_NOFILTERING    = 0,
    VP8_DEBLOCK        = 1,
    VP8_DEMACROBLOCK   = 2,
    VP8_ADDNOISE       = 4
};


typedef struct vp8_postproc_cfg
{
    int post_proc_flag;           /**< the types of post processing to be done, should be combination of "vp8_postproc_level" */
    int deblocking_level;        /**< the strength of deblocking, valid range [0, 16] */
    int noise_level;             /**< the strength of additive noise, valid range [0, 16] */
} vp8_postproc_cfg_t;

typedef enum vpx_ref_frame_type
{
    VP8_LAST_FRAME = 1,
    VP8_GOLD_FRAME = 2,
    VP8_ALTR_FRAME = 4
} vpx_ref_frame_type_t;


typedef struct vpx_ref_frame
{
    vpx_ref_frame_type_t  frame_type;   /**< which reference frame */
    vpx_image_t           img;          /**< reference frame data in image format */
} vpx_ref_frame_t;



VPX_CTRL_USE_TYPE(VP8_SET_REFERENCE,           vpx_ref_frame_t *)
VPX_CTRL_USE_TYPE(VP8_COPY_REFERENCE,          vpx_ref_frame_t *)
VPX_CTRL_USE_TYPE(VP8_SET_POSTPROC,            vp8_postproc_cfg_t *)


/*! @} - end defgroup vp8 */

#if !defined(VPX_CODEC_DISABLE_COMPAT) || !VPX_CODEC_DISABLE_COMPAT

DECLSPEC_DEPRECATED extern vpx_codec_iface_t vpx_codec_vp8_algo DEPRECATED;
#endif

#include "vpx/vpx_codec_impl_bottom.h"
#endif
