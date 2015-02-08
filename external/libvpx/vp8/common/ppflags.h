


#ifndef __INC_PPFLAGS_H
#define __INC_PPFLAGS_H
enum
{
    VP8D_NOFILTERING    = 0,
    VP8D_DEBLOCK        = 1<<0,
    VP8D_DEMACROBLOCK   = 1<<1,
    VP8D_ADDNOISE       = 1<<2,
    VP8D_DEBUG_LEVEL1   = 1<<3,
    VP8D_DEBUG_LEVEL2   = 1<<4,
    VP8D_DEBUG_LEVEL3   = 1<<5,
    VP8D_DEBUG_LEVEL4   = 1<<6,
    VP8D_DEBUG_DRAW_MV          = 1<<7,
    VP8D_DEBUG_CLR_BLK_MODES    = 1<<8,
    VP8D_DEBUG_CLR_FRM_REF_BLKS = 1<<9
};

#endif
