

#ifndef SVGFEDisplacementMap_h
#define SVGFEDisplacementMap_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "PlatformString.h"
#include "FilterEffect.h"
#include "Filter.h"

namespace WebCore {

    enum ChannelSelectorType {
        CHANNEL_UNKNOWN = 0,
        CHANNEL_R       = 1,
        CHANNEL_G       = 2,
        CHANNEL_B       = 3,
        CHANNEL_A       = 4
    };
    
    class FEDisplacementMap : public FilterEffect {
    public:
        static PassRefPtr<FEDisplacementMap> create(FilterEffect*, FilterEffect*, ChannelSelectorType,
                ChannelSelectorType, const float&);

        ChannelSelectorType xChannelSelector() const;
        void setXChannelSelector(const ChannelSelectorType);

        ChannelSelectorType yChannelSelector() const;
        void setYChannelSelector(const ChannelSelectorType);

        float scale() const;
        void setScale(float scale);

        virtual FloatRect uniteChildEffectSubregions(Filter* filter) { return calculateUnionOfChildEffectSubregions(filter, m_in.get(), m_in2.get()); }
        void apply(Filter*);
        void dump();
        TextStream& externalRepresentation(TextStream& ts) const;

    private:
        FEDisplacementMap(FilterEffect*, FilterEffect*, ChannelSelectorType,
            ChannelSelectorType, const float&);

        RefPtr<FilterEffect> m_in;
        RefPtr<FilterEffect> m_in2;
        ChannelSelectorType m_xChannelSelector;
        ChannelSelectorType m_yChannelSelector;
        float m_scale;
    };

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)

#endif // SVGFEDisplacementMap_h
