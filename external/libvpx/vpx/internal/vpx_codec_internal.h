


#ifndef VPX_CODEC_INTERNAL_H
#define VPX_CODEC_INTERNAL_H
#include "../vpx_decoder.h"
#include "../vpx_encoder.h"
#include <stdarg.h>


#define VPX_CODEC_INTERNAL_ABI_VERSION (3) /**<\hideinitializer*/

typedef struct vpx_codec_alg_priv  vpx_codec_alg_priv_t;

typedef vpx_codec_err_t (*vpx_codec_init_fn_t)(vpx_codec_ctx_t *ctx);

typedef vpx_codec_err_t (*vpx_codec_destroy_fn_t)(vpx_codec_alg_priv_t *ctx);

typedef vpx_codec_err_t (*vpx_codec_peek_si_fn_t)(const uint8_t         *data,
        unsigned int           data_sz,
        vpx_codec_stream_info_t *si);

typedef vpx_codec_err_t (*vpx_codec_get_si_fn_t)(vpx_codec_alg_priv_t    *ctx,
        vpx_codec_stream_info_t *si);

typedef vpx_codec_err_t (*vpx_codec_control_fn_t)(vpx_codec_alg_priv_t  *ctx,
        int                  ctrl_id,
        va_list              ap);

typedef const struct
{
    int                    ctrl_id;
    vpx_codec_control_fn_t   fn;
} vpx_codec_ctrl_fn_map_t;

typedef vpx_codec_err_t (*vpx_codec_decode_fn_t)(vpx_codec_alg_priv_t  *ctx,
        const uint8_t         *data,
        unsigned int     data_sz,
        void        *user_priv,
        long         deadline);

typedef vpx_image_t*(*vpx_codec_get_frame_fn_t)(vpx_codec_alg_priv_t *ctx,
        vpx_codec_iter_t     *iter);


typedef vpx_codec_err_t (*vpx_codec_get_mmap_fn_t)(const vpx_codec_ctx_t      *ctx,
        vpx_codec_mmap_t           *mmap,
        vpx_codec_iter_t           *iter);


typedef vpx_codec_err_t (*vpx_codec_set_mmap_fn_t)(vpx_codec_ctx_t         *ctx,
        const vpx_codec_mmap_t  *mmap);


typedef vpx_codec_err_t (*vpx_codec_encode_fn_t)(vpx_codec_alg_priv_t  *ctx,
        const vpx_image_t     *img,
        vpx_codec_pts_t        pts,
        unsigned long          duration,
        vpx_enc_frame_flags_t  flags,
        unsigned long          deadline);
typedef const vpx_codec_cx_pkt_t*(*vpx_codec_get_cx_data_fn_t)(vpx_codec_alg_priv_t *ctx,
        vpx_codec_iter_t     *iter);

typedef vpx_codec_err_t
(*vpx_codec_enc_config_set_fn_t)(vpx_codec_alg_priv_t       *ctx,
                                 const vpx_codec_enc_cfg_t  *cfg);
typedef vpx_fixed_buf_t *
(*vpx_codec_get_global_headers_fn_t)(vpx_codec_alg_priv_t   *ctx);

typedef vpx_image_t *
(*vpx_codec_get_preview_frame_fn_t)(vpx_codec_alg_priv_t   *ctx);

typedef const struct
{
    int                 usage;
    vpx_codec_enc_cfg_t cfg;
} vpx_codec_enc_cfg_map_t;

#define NOT_IMPLEMENTED 0

struct vpx_codec_iface
{
    const char               *name;        /**< Identification String  */
    int                       abi_version; /**< Implemented ABI version */
    vpx_codec_caps_t          caps;    /**< Decoder capabilities */
    vpx_codec_init_fn_t       init;    /**< \copydoc ::vpx_codec_init_fn_t */
    vpx_codec_destroy_fn_t    destroy;     /**< \copydoc ::vpx_codec_destroy_fn_t */
    vpx_codec_ctrl_fn_map_t  *ctrl_maps;   /**< \copydoc ::vpx_codec_ctrl_fn_map_t */
    vpx_codec_get_mmap_fn_t   get_mmap;    /**< \copydoc ::vpx_codec_get_mmap_fn_t */
    vpx_codec_set_mmap_fn_t   set_mmap;    /**< \copydoc ::vpx_codec_set_mmap_fn_t */
    struct
    {
        vpx_codec_peek_si_fn_t    peek_si;     /**< \copydoc ::vpx_codec_peek_si_fn_t */
        vpx_codec_get_si_fn_t     get_si;      /**< \copydoc ::vpx_codec_peek_si_fn_t */
        vpx_codec_decode_fn_t     decode;      /**< \copydoc ::vpx_codec_decode_fn_t */
        vpx_codec_get_frame_fn_t  get_frame;   /**< \copydoc ::vpx_codec_get_frame_fn_t */
    } dec;
    struct
    {
        vpx_codec_enc_cfg_map_t           *cfg_maps;      /**< \copydoc ::vpx_codec_enc_cfg_map_t */
        vpx_codec_encode_fn_t              encode;        /**< \copydoc ::vpx_codec_encode_fn_t */
        vpx_codec_get_cx_data_fn_t         get_cx_data;   /**< \copydoc ::vpx_codec_get_cx_data_fn_t */
        vpx_codec_enc_config_set_fn_t      cfg_set;       /**< \copydoc ::vpx_codec_enc_config_set_fn_t */
        vpx_codec_get_global_headers_fn_t  get_glob_hdrs; /**< \copydoc ::vpx_codec_enc_config_set_fn_t */
        vpx_codec_get_preview_frame_fn_t   get_preview;   /**< \copydoc ::vpx_codec_get_preview_frame_fn_t */
    } enc;
};

/*!\brief Callback function pointer / user data pair storage */
typedef struct vpx_codec_priv_cb_pair
{
    union
    {
        vpx_codec_put_frame_cb_fn_t    put_frame;
        vpx_codec_put_slice_cb_fn_t    put_slice;
    };
    void                            *user_priv;
} vpx_codec_priv_cb_pair_t;


struct vpx_codec_priv
{
    unsigned int                    sz;
    vpx_codec_iface_t              *iface;
    struct vpx_codec_alg_priv      *alg_priv;
    const char                     *err_detail;
    vpx_codec_flags_t               init_flags;
    struct
    {
        vpx_codec_priv_cb_pair_t    put_frame_cb;
        vpx_codec_priv_cb_pair_t    put_slice_cb;
    } dec;
    struct
    {
        int                         tbd;
        struct vpx_fixed_buf        cx_data_dst_buf;
        unsigned int                cx_data_pad_before;
        unsigned int                cx_data_pad_after;
        vpx_codec_cx_pkt_t          cx_data_pkt;
    } enc;
};

#undef VPX_CTRL_USE_TYPE
#define VPX_CTRL_USE_TYPE(id, typ) \
    static typ id##__value(va_list args) {return va_arg(args, typ);} \
    static typ id##__convert(void *x)\
    {\
        union\
        {\
            void *x;\
            typ   d;\
        } u;\
        u.x = x;\
        return u.d;\
    }


#undef VPX_CTRL_USE_TYPE_DEPRECATED
#define VPX_CTRL_USE_TYPE_DEPRECATED(id, typ) \
    static typ id##__value(va_list args) {return va_arg(args, typ);} \
    static typ id##__convert(void *x)\
    {\
        union\
        {\
            void *x;\
            typ   d;\
        } u;\
        u.x = x;\
        return u.d;\
    }

#define CAST(id, arg) id##__value(arg)
#define RECAST(id, x) id##__convert(x)


#define CODEC_INTERFACE(id)\
vpx_codec_iface_t* id(void) { return &id##_algo; }\
vpx_codec_iface_t  id##_algo


struct vpx_codec_pkt_list
{
    unsigned int            cnt;
    unsigned int            max;
    struct vpx_codec_cx_pkt pkts[1];
};

#define vpx_codec_pkt_list_decl(n)\
    union {struct vpx_codec_pkt_list head;\
        struct {struct vpx_codec_pkt_list head;\
            struct vpx_codec_cx_pkt    pkts[n];} alloc;}

#define vpx_codec_pkt_list_init(m)\
    (m)->alloc.head.cnt = 0,\
                          (m)->alloc.head.max = sizeof((m)->alloc.pkts) / sizeof((m)->alloc.pkts[0])

int
vpx_codec_pkt_list_add(struct vpx_codec_pkt_list *,
                       const struct vpx_codec_cx_pkt *);

const vpx_codec_cx_pkt_t*
vpx_codec_pkt_list_get(struct vpx_codec_pkt_list *list,
                       vpx_codec_iter_t           *iter);


#include <stdio.h>
#include <setjmp.h>
struct vpx_internal_error_info
{
    vpx_codec_err_t  error_code;
    int              has_detail;
    char             detail[80];
    int              setjmp;
    jmp_buf          jmp;
};

static void vpx_internal_error(struct vpx_internal_error_info *info,
                               vpx_codec_err_t                 error,
                               const char                     *fmt,
                               ...)
{
    va_list ap;

    info->error_code = error;
    info->has_detail = 0;

    if (fmt)
    {
        size_t  sz = sizeof(info->detail);

        info->has_detail = 1;
        va_start(ap, fmt);
        vsnprintf(info->detail, sz - 1, fmt, ap);
        va_end(ap);
        info->detail[sz-1] = '\0';
    }

    if (info->setjmp)
        longjmp(info->jmp, info->error_code);
}
#endif
