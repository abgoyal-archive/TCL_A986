



#include "vpx_scale/vpxscale.h"


static
void vp8cx_horizontal_line_4_5_scale_c
(
    const unsigned char *source,
    unsigned int source_width,
    unsigned char *dest,
    unsigned int dest_width
)
{
    unsigned i;
    unsigned int a, b, c;
    unsigned char *des = dest;
    const unsigned char *src = source;

    (void) dest_width;

    for (i = 0; i < source_width - 4; i += 4)
    {
        a = src[0];
        b = src[1];
        des [0] = (unsigned char) a;
        des [1] = (unsigned char)((a * 51 + 205 * b + 128) >> 8);
        c = src[2] * 154;
        a = src[3];
        des [2] = (unsigned char)((b * 102 + c + 128) >> 8);
        des [3] = (unsigned char)((c + 102 * a + 128) >> 8);
        b = src[4];
        des [4] = (unsigned char)((a * 205 + 51 * b + 128) >> 8);

        src += 4;
        des += 5;
    }

    a = src[0];
    b = src[1];
    des [0] = (unsigned char)(a);
    des [1] = (unsigned char)((a * 51 + 205 * b + 128) >> 8);
    c = src[2] * 154;
    a = src[3];
    des [2] = (unsigned char)((b * 102 + c + 128) >> 8);
    des [3] = (unsigned char)((c + 102 * a + 128) >> 8);
    des [4] = (unsigned char)(a);

}

static
void vp8cx_vertical_band_4_5_scale_c(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width)
{
    unsigned int i;
    unsigned int a, b, c, d;
    unsigned char *des = dest;

    for (i = 0; i < dest_width; i++)
    {
        a = des [0];
        b = des [dest_pitch];

        des[dest_pitch] = (unsigned char)((a * 51 + 205 * b + 128) >> 8);

        c = des[dest_pitch*2] * 154;
        d = des[dest_pitch*3];

        des [dest_pitch*2] = (unsigned char)((b * 102 + c + 128) >> 8);
        des [dest_pitch*3] = (unsigned char)((c + 102 * d + 128) >> 8);

        // First line in next band
        a = des [dest_pitch * 5];
        des [dest_pitch * 4] = (unsigned char)((d * 205 + 51 * a + 128) >> 8);

        des ++;
    }
}

static
void vp8cx_last_vertical_band_4_5_scale_c(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width)
{
    unsigned int i;
    unsigned int a, b, c, d;
    unsigned char *des = dest;

    for (i = 0; i < dest_width; ++i)
    {
        a = des[0];
        b = des[dest_pitch];

        des[dest_pitch] = (unsigned char)((a * 51 + 205 * b + 128) >> 8);

        c = des[dest_pitch*2] * 154;
        d = des[dest_pitch*3];

        des [dest_pitch*2] = (unsigned char)((b * 102 + c + 128) >> 8);
        des [dest_pitch*3] = (unsigned char)((c + 102 * d + 128) >> 8);

        // No other line for interplation of this line, so ..
        des[dest_pitch*4] = (unsigned char) d;

        des++;
    }
}

static
void vp8cx_horizontal_line_3_5_scale_c
(
    const unsigned char *source,
    unsigned int source_width,
    unsigned char *dest,
    unsigned int dest_width
)
{
    unsigned int i;
    unsigned int a, b, c;
    unsigned char *des = dest;
    const unsigned char *src = source;

    (void) dest_width;

    for (i = 0; i < source_width - 3; i += 3)
    {
        a = src[0];
        b = src[1];
        des [0] = (unsigned char)(a);
        des [1] = (unsigned char)((a * 102 + 154 * b + 128) >> 8);

        c = src[2] ;
        des [2] = (unsigned char)((b * 205 + c * 51 + 128) >> 8);
        des [3] = (unsigned char)((b * 51 + c * 205 + 128) >> 8);

        a = src[3];
        des [4] = (unsigned char)((c * 154 + a * 102 + 128) >> 8);

        src += 3;
        des += 5;
    }

    a = src[0];
    b = src[1];
    des [0] = (unsigned char)(a);

    des [1] = (unsigned char)((a * 102 + 154 * b + 128) >> 8);
    c = src[2] ;
    des [2] = (unsigned char)((b * 205 + c * 51 + 128) >> 8);
    des [3] = (unsigned char)((b * 51 + c * 205 + 128) >> 8);

    des [4] = (unsigned char)(c);
}

static
void vp8cx_vertical_band_3_5_scale_c(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width)
{
    unsigned int i;
    unsigned int a, b, c;
    unsigned char *des = dest;

    for (i = 0; i < dest_width; i++)
    {
        a = des [0];
        b = des [dest_pitch];
        des [dest_pitch] = (unsigned char)((a * 102 + 154 * b + 128) >> 8);

        c = des[dest_pitch*2];
        des [dest_pitch*2] = (unsigned char)((b * 205 + c * 51 + 128) >> 8);
        des [dest_pitch*3] = (unsigned char)((b * 51 + c * 205 + 128) >> 8);

        // First line in next band...
        a = des [dest_pitch * 5];
        des [dest_pitch * 4] = (unsigned char)((c * 154 + a * 102 + 128) >> 8);

        des++;
    }
}

static
void vp8cx_last_vertical_band_3_5_scale_c(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width)
{
    unsigned int i;
    unsigned int a, b, c;
    unsigned char *des = dest;

    for (i = 0; i < dest_width; ++i)
    {
        a = des [0];
        b = des [dest_pitch];

        des [ dest_pitch ] = (unsigned char)((a * 102 + 154 * b + 128) >> 8);

        c = des[dest_pitch*2];
        des [dest_pitch*2] = (unsigned char)((b * 205 + c * 51 + 128) >> 8);
        des [dest_pitch*3] = (unsigned char)((b * 51 + c * 205 + 128) >> 8);

        // No other line for interplation of this line, so ..
        des [ dest_pitch * 4 ] = (unsigned char)(c) ;

        des++;
    }
}

static
void vp8cx_horizontal_line_1_2_scale_c
(
    const unsigned char *source,
    unsigned int source_width,
    unsigned char *dest,
    unsigned int dest_width
)
{
    unsigned int i;
    unsigned int a, b;
    unsigned char *des = dest;
    const unsigned char *src = source;

    (void) dest_width;

    for (i = 0; i < source_width - 1; i += 1)
    {
        a = src[0];
        b = src[1];
        des [0] = (unsigned char)(a);
        des [1] = (unsigned char)((a + b + 1) >> 1);
        src += 1;
        des += 2;
    }

    a = src[0];
    des [0] = (unsigned char)(a);
    des [1] = (unsigned char)(a);
}

static
void vp8cx_vertical_band_1_2_scale_c(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width)
{
    unsigned int i;
    unsigned int a, b;
    unsigned char *des = dest;

    for (i = 0; i < dest_width; i++)
    {
        a = des [0];
        b = des [dest_pitch * 2];

        des[dest_pitch] = (unsigned char)((a + b + 1) >> 1);

        des++;
    }
}

static
void vp8cx_last_vertical_band_1_2_scale_c(unsigned char *dest, unsigned int dest_pitch, unsigned int dest_width)
{
    unsigned int i;
    unsigned char *des = dest;

    for (i = 0; i < dest_width; ++i)
    {
        des[dest_pitch] = des[0];
        des++;
    }
}

#include "vpx_scale/vpxscale.h"
#include "vpx_mem/vpx_mem.h"

struct vpxglobal_scalling_ptrs_t *g_scaling_ptrs = 0;

int
register_generic_scalers(void)
{
    int rv = 0;

    g_scaling_ptrs = (struct vpxglobal_scalling_ptrs_t *)vpx_malloc(sizeof(struct vpxglobal_scalling_ptrs_t));

    if (g_scaling_ptrs)
    {
        g_scaling_ptrs->vpxhorizontal_line_1_2_scale_t        = vp8cx_horizontal_line_1_2_scale_c;
        g_scaling_ptrs->vpxvertical_band_1_2_scale_t          = vp8cx_vertical_band_1_2_scale_c;
        g_scaling_ptrs->vpxlast_vertical_band_1_2_scale_t      = vp8cx_last_vertical_band_1_2_scale_c;
        g_scaling_ptrs->vpxhorizontal_line_3_5_scale_t        = vp8cx_horizontal_line_3_5_scale_c;
        g_scaling_ptrs->vpxvertical_band_3_5_scale_t          = vp8cx_vertical_band_3_5_scale_c;
        g_scaling_ptrs->vpxlast_vertical_band_3_5_scale_t      = vp8cx_last_vertical_band_3_5_scale_c;
        g_scaling_ptrs->vpxhorizontal_line_4_5_scale_t        = vp8cx_horizontal_line_4_5_scale_c;
        g_scaling_ptrs->vpxvertical_band_4_5_scale_t          = vp8cx_vertical_band_4_5_scale_c;
        g_scaling_ptrs->vpxlast_vertical_band_4_5_scale_t      = vp8cx_last_vertical_band_4_5_scale_c;
    }
    else
    {
        rv = -1;
    }

    /*
    vp8_horizontal_line_1_2_scale        = vp8cx_horizontal_line_1_2_scale_c;
    vp8_vertical_band_1_2_scale          = vp8cx_vertical_band_1_2_scale_c;
    vp8_last_vertical_band_1_2_scale      = vp8cx_last_vertical_band_1_2_scale_c;
    vp8_horizontal_line_3_5_scale        = vp8cx_horizontal_line_3_5_scale_c;
    vp8_vertical_band_3_5_scale          = vp8cx_vertical_band_3_5_scale_c;
    vp8_last_vertical_band_3_5_scale      = vp8cx_last_vertical_band_3_5_scale_c;
    vp8_horizontal_line_4_5_scale        = vp8cx_horizontal_line_4_5_scale_c;
    vp8_vertical_band_4_5_scale          = vp8cx_vertical_band_4_5_scale_c;
    vp8_last_vertical_band_4_5_scale      = vp8cx_last_vertical_band_4_5_scale_c;
    */

    return rv;
}

int
de_register_generic_scalers(void)
{
    int rv = 0;

    if (g_scaling_ptrs)
    {
        vpx_free(g_scaling_ptrs);
        g_scaling_ptrs = 0;
    }
    else
    {
        rv = -1;
    }

    return rv;
}
