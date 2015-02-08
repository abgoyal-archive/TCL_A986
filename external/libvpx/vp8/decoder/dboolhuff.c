


#include "dboolhuff.h"
#include "vpx_ports/mem.h"
#include "vpx_mem/vpx_mem.h"

DECLARE_ALIGNED(16, const unsigned char, vp8dx_bitreader_norm[256]) =
{
    0, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};


int vp8dx_start_decode_c(BOOL_DECODER *br, const unsigned char *source,
                        unsigned int source_sz)
{
    br->user_buffer_end = source+source_sz;
    br->user_buffer     = source;
    br->value    = 0;
    br->count    = -8;
    br->range    = 255;

    if (source_sz && !source)
        return 1;

    /* Populate the buffer */
    vp8dx_bool_decoder_fill_c(br);

    return 0;
}


void vp8dx_bool_decoder_fill_c(BOOL_DECODER *br)
{
    const unsigned char *bufptr;
    const unsigned char *bufend;
    VP8_BD_VALUE         value;
    int                  count;
    bufend = br->user_buffer_end;
    bufptr = br->user_buffer;
    value = br->value;
    count = br->count;

    VP8DX_BOOL_DECODER_FILL(count, value, bufptr, bufend);

    br->user_buffer = bufptr;
    br->value = value;
    br->count = count;
}

#if 0
int vp8dx_decode_bool_c(BOOL_DECODER *br, int probability)
{
    unsigned int bit=0;
    VP8_BD_VALUE value;
    unsigned int split;
    VP8_BD_VALUE bigsplit;
    int count;
    unsigned int range;

    value = br->value;
    count = br->count;
    range = br->range;

    split = 1 + (((range-1) * probability) >> 8);
    bigsplit = (VP8_BD_VALUE)split << (VP8_BD_VALUE_SIZE - 8);

    range = split;
    if(value >= bigsplit)
    {
        range = br->range-split;
        value = value-bigsplit;
        bit = 1;
    }

    /*if(range>=0x80)
    {
        br->value = value;
        br->range = range;
        return bit;
    }*/

    {
        register unsigned int shift = vp8dx_bitreader_norm[range];
        range <<= shift;
        value <<= shift;
        count -= shift;
    }
    br->value = value;
    br->count = count;
    br->range = range;
    if (count < 0)
        vp8dx_bool_decoder_fill_c(br);
    return bit;
}

int vp8dx_decode_value_c(BOOL_DECODER *br, int bits)
{
    int z = 0;
    int bit;
    for ( bit=bits-1; bit>=0; bit-- )
    {
        z |= (vp8dx_decode_bool(br, 0x80)<<bit);
    }
    return z;
}
#endif
