
#include "qemu-common.h"
#include "audio.h"

#define AUDIO_CAP "mixeng"
#include "audio_int.h"

/* 8 bit */
#define ENDIAN_CONVERSION natural
#define ENDIAN_CONVERT(v) (v)

/* Signed 8 bit */
#define IN_T int8_t
#define IN_MIN SCHAR_MIN
#define IN_MAX SCHAR_MAX
#define SIGNED
#define SHIFT 8
#include "mixeng_template.h"
#undef SIGNED
#undef IN_MAX
#undef IN_MIN
#undef IN_T
#undef SHIFT

/* Unsigned 8 bit */
#define IN_T uint8_t
#define IN_MIN 0
#define IN_MAX UCHAR_MAX
#define SHIFT 8
#include "mixeng_template.h"
#undef IN_MAX
#undef IN_MIN
#undef IN_T
#undef SHIFT

#undef ENDIAN_CONVERT
#undef ENDIAN_CONVERSION

/* Signed 16 bit */
#define IN_T int16_t
#define IN_MIN SHRT_MIN
#define IN_MAX SHRT_MAX
#define SIGNED
#define SHIFT 16
#define ENDIAN_CONVERSION natural
#define ENDIAN_CONVERT(v) (v)
#include "mixeng_template.h"
#undef ENDIAN_CONVERT
#undef ENDIAN_CONVERSION
#define ENDIAN_CONVERSION swap
#define ENDIAN_CONVERT(v) bswap16 (v)
#include "mixeng_template.h"
#undef ENDIAN_CONVERT
#undef ENDIAN_CONVERSION
#undef SIGNED
#undef IN_MAX
#undef IN_MIN
#undef IN_T
#undef SHIFT

/* Unsigned 16 bit */
#define IN_T uint16_t
#define IN_MIN 0
#define IN_MAX USHRT_MAX
#define SHIFT 16
#define ENDIAN_CONVERSION natural
#define ENDIAN_CONVERT(v) (v)
#include "mixeng_template.h"
#undef ENDIAN_CONVERT
#undef ENDIAN_CONVERSION
#define ENDIAN_CONVERSION swap
#define ENDIAN_CONVERT(v) bswap16 (v)
#include "mixeng_template.h"
#undef ENDIAN_CONVERT
#undef ENDIAN_CONVERSION
#undef IN_MAX
#undef IN_MIN
#undef IN_T
#undef SHIFT

/* Signed 32 bit */
#define IN_T int32_t
#define IN_MIN INT32_MIN
#define IN_MAX INT32_MAX
#define SIGNED
#define SHIFT 32
#define ENDIAN_CONVERSION natural
#define ENDIAN_CONVERT(v) (v)
#include "mixeng_template.h"
#undef ENDIAN_CONVERT
#undef ENDIAN_CONVERSION
#define ENDIAN_CONVERSION swap
#define ENDIAN_CONVERT(v) bswap32 (v)
#include "mixeng_template.h"
#undef ENDIAN_CONVERT
#undef ENDIAN_CONVERSION
#undef SIGNED
#undef IN_MAX
#undef IN_MIN
#undef IN_T
#undef SHIFT

/* Unsigned 16 bit */
#define IN_T uint32_t
#define IN_MIN 0
#define IN_MAX UINT32_MAX
#define SHIFT 32
#define ENDIAN_CONVERSION natural
#define ENDIAN_CONVERT(v) (v)
#include "mixeng_template.h"
#undef ENDIAN_CONVERT
#undef ENDIAN_CONVERSION
#define ENDIAN_CONVERSION swap
#define ENDIAN_CONVERT(v) bswap32 (v)
#include "mixeng_template.h"
#undef ENDIAN_CONVERT
#undef ENDIAN_CONVERSION
#undef IN_MAX
#undef IN_MIN
#undef IN_T
#undef SHIFT

t_sample *mixeng_conv[2][2][2][3] = {
    {
        {
            {
                conv_natural_uint8_t_to_mono,
                conv_natural_uint16_t_to_mono,
                conv_natural_uint32_t_to_mono
            },
            {
                conv_natural_uint8_t_to_mono,
                conv_swap_uint16_t_to_mono,
                conv_swap_uint32_t_to_mono,
            }
        },
        {
            {
                conv_natural_int8_t_to_mono,
                conv_natural_int16_t_to_mono,
                conv_natural_int32_t_to_mono
            },
            {
                conv_natural_int8_t_to_mono,
                conv_swap_int16_t_to_mono,
                conv_swap_int32_t_to_mono
            }
        }
    },
    {
        {
            {
                conv_natural_uint8_t_to_stereo,
                conv_natural_uint16_t_to_stereo,
                conv_natural_uint32_t_to_stereo
            },
            {
                conv_natural_uint8_t_to_stereo,
                conv_swap_uint16_t_to_stereo,
                conv_swap_uint32_t_to_stereo
            }
        },
        {
            {
                conv_natural_int8_t_to_stereo,
                conv_natural_int16_t_to_stereo,
                conv_natural_int32_t_to_stereo
            },
            {
                conv_natural_int8_t_to_stereo,
                conv_swap_int16_t_to_stereo,
                conv_swap_int32_t_to_stereo,
            }
        }
    }
};

f_sample *mixeng_clip[2][2][2][3] = {
    {
        {
            {
                clip_natural_uint8_t_from_mono,
                clip_natural_uint16_t_from_mono,
                clip_natural_uint32_t_from_mono
            },
            {
                clip_natural_uint8_t_from_mono,
                clip_swap_uint16_t_from_mono,
                clip_swap_uint32_t_from_mono
            }
        },
        {
            {
                clip_natural_int8_t_from_mono,
                clip_natural_int16_t_from_mono,
                clip_natural_int32_t_from_mono
            },
            {
                clip_natural_int8_t_from_mono,
                clip_swap_int16_t_from_mono,
                clip_swap_int32_t_from_mono
            }
        }
    },
    {
        {
            {
                clip_natural_uint8_t_from_stereo,
                clip_natural_uint16_t_from_stereo,
                clip_natural_uint32_t_from_stereo
            },
            {
                clip_natural_uint8_t_from_stereo,
                clip_swap_uint16_t_from_stereo,
                clip_swap_uint32_t_from_stereo
            }
        },
        {
            {
                clip_natural_int8_t_from_stereo,
                clip_natural_int16_t_from_stereo,
                clip_natural_int32_t_from_stereo
            },
            {
                clip_natural_int8_t_from_stereo,
                clip_swap_int16_t_from_stereo,
                clip_swap_int32_t_from_stereo
            }
        }
    }
};



/* Private data */
struct rate {
    uint64_t opos;
    uint64_t opos_inc;
    uint32_t ipos;              /* position in the input stream (integer) */
    struct st_sample ilast;          /* last sample in the input stream */
};

void *st_rate_start (int inrate, int outrate)
{
    struct rate *rate = audio_calloc (AUDIO_FUNC, 1, sizeof (*rate));

    if (!rate) {
        dolog ("Could not allocate resampler (%zu bytes)\n", sizeof (*rate));
        return NULL;
    }

    rate->opos = 0;

    /* increment */
    rate->opos_inc = ((uint64_t) inrate << 32) / outrate;

    rate->ipos = 0;
    rate->ilast.l = 0;
    rate->ilast.r = 0;
    return rate;
}

#define NAME st_rate_flow_mix
#define OP(a, b) a += b
#include "rate_template.h"

#define NAME st_rate_flow
#define OP(a, b) a = b
#include "rate_template.h"

void st_rate_stop (void *opaque)
{
    qemu_free (opaque);
}

void mixeng_clear (struct st_sample *buf, int len)
{
    memset (buf, 0, len * sizeof (struct st_sample));
}
