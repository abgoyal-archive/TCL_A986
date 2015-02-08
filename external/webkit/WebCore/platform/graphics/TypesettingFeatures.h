

#ifndef TypesettingFeatures_h
#define TypesettingFeatures_h

namespace WebCore {
    enum TypesettingFeature {
        Kerning = 1 << 0,
        Ligatures = 1 << 1,
    };

    typedef unsigned TypesettingFeatures;
} // namespace WebCore

#endif // TypesettingFeatures_h
