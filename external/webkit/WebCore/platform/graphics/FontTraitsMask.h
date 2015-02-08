

#ifndef FontTraitsMask_h
#define FontTraitsMask_h

namespace WebCore {

    enum {
        FontStyleNormalBit = 0,
        FontStyleItalicBit,
        FontVariantNormalBit,
        FontVariantSmallCapsBit,
        FontWeight100Bit,
        FontWeight200Bit,
        FontWeight300Bit,
        FontWeight400Bit,
        FontWeight500Bit,
        FontWeight600Bit,
        FontWeight700Bit,
        FontWeight800Bit,
        FontWeight900Bit,
        FontTraitsMaskWidth
    };

    enum FontTraitsMask {
        FontStyleNormalMask = 1 << FontStyleNormalBit,
        FontStyleItalicMask = 1 << FontStyleItalicBit,
        FontStyleMask = FontStyleNormalMask | FontStyleItalicMask,

        FontVariantNormalMask = 1 << FontVariantNormalBit,
        FontVariantSmallCapsMask = 1 << FontVariantSmallCapsBit,
        FontVariantMask = FontVariantNormalMask | FontVariantSmallCapsMask,

        FontWeight100Mask = 1 << FontWeight100Bit,
        FontWeight200Mask = 1 << FontWeight200Bit,
        FontWeight300Mask = 1 << FontWeight300Bit,
        FontWeight400Mask = 1 << FontWeight400Bit,
        FontWeight500Mask = 1 << FontWeight500Bit,
        FontWeight600Mask = 1 << FontWeight600Bit,
        FontWeight700Mask = 1 << FontWeight700Bit,
        FontWeight800Mask = 1 << FontWeight800Bit,
        FontWeight900Mask = 1 << FontWeight900Bit,
        FontWeightMask = FontWeight100Mask | FontWeight200Mask | FontWeight300Mask | FontWeight400Mask | FontWeight500Mask | FontWeight600Mask | FontWeight700Mask | FontWeight800Mask | FontWeight900Mask
    };

} // namespace WebCore
#endif // FontTraitsMask_h
