


#include "vp8.h"

#ifndef VP8CX_H
#define VP8CX_H
#include "vpx/vpx_codec_impl_top.h"

extern vpx_codec_iface_t  vpx_codec_vp8_cx_algo;
extern vpx_codec_iface_t* vpx_codec_vp8_cx(void);



#define VP8_EFLAG_NO_REF_LAST      (1<<16)


#define VP8_EFLAG_NO_REF_GF        (1<<17)


#define VP8_EFLAG_NO_REF_ARF       (1<<21)


#define VP8_EFLAG_NO_UPD_LAST      (1<<18)


#define VP8_EFLAG_NO_UPD_GF        (1<<22)


#define VP8_EFLAG_NO_UPD_ARF       (1<<23)


#define VP8_EFLAG_FORCE_GF         (1<<19)


#define VP8_EFLAG_FORCE_ARF        (1<<24)


#define VP8_EFLAG_NO_UPD_ENTROPY   (1<<20)


enum vp8e_enc_control_id
{
    VP8E_UPD_ENTROPY           = 5,  /**< control function to set mode of entropy update in encoder */
    VP8E_UPD_REFERENCE,              /**< control function to set reference update mode in encoder */
    VP8E_USE_REFERENCE,              /**< control function to set which reference frame encoder can use */
    VP8E_SET_ROI_MAP,                /**< control function to pass an ROI map to encoder */
    VP8E_SET_ACTIVEMAP,              /**< control function to pass an Active map to encoder */
    VP8E_SET_SCALEMODE         = 11, /**< control function to set encoder scaling mode */
    VP8E_SET_CPUUSED           = 13, /**< control function to set vp8 encoder cpuused  */
    VP8E_SET_ENABLEAUTOALTREF,       /**< control function to enable vp8 to automatic set and use altref frame */
    VP8E_SET_NOISE_SENSITIVITY,      /**< control function to set noise sensitivity */
    VP8E_SET_SHARPNESS,              /**< control function to set sharpness */
    VP8E_SET_STATIC_THRESHOLD,       /**< control function to set the threshold for macroblocks treated static */
    VP8E_SET_TOKEN_PARTITIONS,       /**< control function to set the number of token partitions  */
    VP8E_GET_LAST_QUANTIZER,         /**< return the quantizer chosen by the
                                          encoder for the last frame using the internal
                                          scale */
    VP8E_GET_LAST_QUANTIZER_64,      /**< return the quantizer chosen by the
                                          encoder for the last frame, using the 0..63
                                          scale as used by the rc_*_quantizer config
                                          parameters */
    VP8E_SET_ARNR_MAXFRAMES,         /**< control function to set the max number of frames blurred creating arf*/
    VP8E_SET_ARNR_STRENGTH ,         /**< control function to set the filter strength for the arf */
    VP8E_SET_ARNR_TYPE     ,         /**< control function to set the type of filter to use for the arf*/
} ;

typedef enum vpx_scaling_mode_1d
{
    VP8E_NORMAL      = 0,
    VP8E_FOURFIVE    = 1,
    VP8E_THREEFIVE   = 2,
    VP8E_ONETWO      = 3
} VPX_SCALING_MODE;



typedef struct vpx_roi_map
{
    unsigned char *roi_map;      /**< specify an id between 0 and 3 for each 16x16 region within a frame */
    unsigned int   rows;         /**< number of rows */
    unsigned int   cols;         /**< number of cols */
    int     delta_q[4];          /**< quantizer delta [-64, 64] off baseline for regions with id between 0 and 3*/
    int     delta_lf[4];         /**< loop filter strength delta [-32, 32] for regions with id between 0 and 3 */
    unsigned int   static_threshold[4];/**< threshold for region to be treated as static */
} vpx_roi_map_t;



typedef struct vpx_active_map
{
    unsigned char  *active_map; /**< specify an on (1) or off (0) each 16x16 region within a frame */
    unsigned int    rows;       /**< number of rows */
    unsigned int    cols;       /**< number of cols */
} vpx_active_map_t;

typedef struct vpx_scaling_mode
{
    VPX_SCALING_MODE    h_scaling_mode;  /**< horizontal scaling mode */
    VPX_SCALING_MODE    v_scaling_mode;  /**< vertical scaling mode   */
} vpx_scaling_mode_t;

typedef enum
{
    VP8_BEST_QUALITY_ENCODING,
    VP8_GOOD_QUALITY_ENCODING,
    VP8_REAL_TIME_ENCODING
} vp8e_encoding_mode;


typedef enum
{
    VP8_ONE_TOKENPARTITION   = 0,
    VP8_TWO_TOKENPARTITION   = 1,
    VP8_FOUR_TOKENPARTITION  = 2,
    VP8_EIGHT_TOKENPARTITION = 3,
} vp8e_token_partitions;




VPX_CTRL_USE_TYPE_DEPRECATED(VP8E_UPD_ENTROPY,            int)
VPX_CTRL_USE_TYPE_DEPRECATED(VP8E_UPD_REFERENCE,          int)
VPX_CTRL_USE_TYPE_DEPRECATED(VP8E_USE_REFERENCE,          int)

VPX_CTRL_USE_TYPE(VP8E_SET_ROI_MAP,            vpx_roi_map_t *)
VPX_CTRL_USE_TYPE(VP8E_SET_ACTIVEMAP,          vpx_active_map_t *)
VPX_CTRL_USE_TYPE(VP8E_SET_SCALEMODE,          vpx_scaling_mode_t *)

VPX_CTRL_USE_TYPE(VP8E_SET_CPUUSED,            int)
VPX_CTRL_USE_TYPE(VP8E_SET_ENABLEAUTOALTREF,   unsigned int)
VPX_CTRL_USE_TYPE(VP8E_SET_NOISE_SENSITIVITY,  unsigned int)
VPX_CTRL_USE_TYPE(VP8E_SET_SHARPNESS,          unsigned int)
VPX_CTRL_USE_TYPE(VP8E_SET_STATIC_THRESHOLD,   unsigned int)
VPX_CTRL_USE_TYPE(VP8E_SET_TOKEN_PARTITIONS,   vp8e_token_partitions)

VPX_CTRL_USE_TYPE(VP8E_SET_ARNR_MAXFRAMES,     unsigned int)
VPX_CTRL_USE_TYPE(VP8E_SET_ARNR_STRENGTH ,     unsigned int)
VPX_CTRL_USE_TYPE(VP8E_SET_ARNR_TYPE     ,     unsigned int)


VPX_CTRL_USE_TYPE(VP8E_GET_LAST_QUANTIZER,     int *)
VPX_CTRL_USE_TYPE(VP8E_GET_LAST_QUANTIZER_64,  int *)

/*! @} - end defgroup vp8_encoder */
#include "vpx/vpx_codec_impl_bottom.h"
#endif
