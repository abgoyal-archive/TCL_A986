

#ifndef SVGFETurbulence_h
#define SVGFETurbulence_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "FilterEffect.h"
#include "Filter.h"

namespace WebCore {

    enum TurbulanceType {
        FETURBULENCE_TYPE_UNKNOWN      = 0,
        FETURBULENCE_TYPE_FRACTALNOISE = 1,
        FETURBULENCE_TYPE_TURBULENCE   = 2
    };

    class FETurbulence : public FilterEffect {
    public:
        static PassRefPtr<FETurbulence> create(TurbulanceType, const float&, const float&, const int&, const float&,
                bool);

        TurbulanceType type() const;
        void setType(TurbulanceType);

        float baseFrequencyY() const;
        void setBaseFrequencyY(float);

        float baseFrequencyX() const;
        void setBaseFrequencyX(float);

        float seed() const;
        void setSeed(float);

        int numOctaves() const;
        void setNumOctaves(bool);

        bool stitchTiles() const;
        void setStitchTiles(bool);

        void apply(Filter*);
        void dump();
        TextStream& externalRepresentation(TextStream& ts) const;

    private:
        FETurbulence(TurbulanceType, const float&, const float&, const int&, const float&,
                bool);

        TurbulanceType m_type;
        float m_baseFrequencyX;
        float m_baseFrequencyY;
        int m_numOctaves;
        float m_seed;
        bool m_stitchTiles;
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)

#endif // SVGFETurbulence_h
