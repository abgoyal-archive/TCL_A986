

// must include config.h first for webkit to fiddle with new/delete
#include "config.h"
#include "SkANP.h"
#include "SkColorPriv.h"

static bool anp_getPixelPacking(ANPBitmapFormat fmt, ANPPixelPacking* packing) {
    switch (fmt) {
        case kRGBA_8888_ANPBitmapFormat:
            if (packing) {
                packing->AShift = SK_A32_SHIFT;
                packing->ABits  = SK_A32_BITS;
                packing->RShift = SK_R32_SHIFT;
                packing->RBits  = SK_R32_BITS;
                packing->GShift = SK_G32_SHIFT;
                packing->GBits  = SK_G32_BITS;
                packing->BShift = SK_B32_SHIFT;
                packing->BBits  = SK_B32_BITS;
            }
            return true;
        case kRGB_565_ANPBitmapFormat:
            if (packing) {
                packing->AShift = 0;
                packing->ABits  = 0;
                packing->RShift = SK_R16_SHIFT;
                packing->RBits  = SK_R16_BITS;
                packing->GShift = SK_G16_SHIFT;
                packing->GBits  = SK_G16_BITS;
                packing->BShift = SK_B16_SHIFT;
                packing->BBits  = SK_B16_BITS;
            }
            return true;
        default:
            break;
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////

#define ASSIGN(obj, name)   (obj)->name = anp_##name

void ANPBitmapInterfaceV0_Init(ANPInterface* value) {
    ANPBitmapInterfaceV0* i = reinterpret_cast<ANPBitmapInterfaceV0*>(value);

    ASSIGN(i, getPixelPacking);
}
