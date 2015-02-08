

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEMerge.h"

#include "Filter.h"
#include "GraphicsContext.h"
#include "SVGRenderTreeAsText.h"

namespace WebCore {

FEMerge::FEMerge(const Vector<RefPtr<FilterEffect> >& mergeInputs) 
    : FilterEffect()
    , m_mergeInputs(mergeInputs)
{
}

PassRefPtr<FEMerge> FEMerge::create(const Vector<RefPtr<FilterEffect> >& mergeInputs)
{
    return adoptRef(new FEMerge(mergeInputs));
}

const Vector<RefPtr<FilterEffect> >& FEMerge::mergeInputs() const
{
    return m_mergeInputs;
}

void FEMerge::setMergeInputs(const Vector<RefPtr<FilterEffect> >& mergeInputs)
{
    m_mergeInputs = mergeInputs;
}

FloatRect FEMerge::uniteChildEffectSubregions(Filter* filter)
{
    ASSERT(!m_mergeInputs.isEmpty());

    FloatRect uniteEffectRect = m_mergeInputs[0]->calculateEffectRect(filter);

    for (unsigned i = 1; i < m_mergeInputs.size(); i++)
        uniteEffectRect.unite(m_mergeInputs[i]->calculateEffectRect(filter));

    return uniteEffectRect;
}

void FEMerge::apply(Filter* filter)
{
    ASSERT(!m_mergeInputs.isEmpty());

    for (unsigned i = 0; i < m_mergeInputs.size(); i++) {
        m_mergeInputs[i]->apply(filter);
        if (!m_mergeInputs[i]->resultImage())
            return;
    }

    GraphicsContext* filterContext = getEffectContext();
    if (!filterContext)
        return;

    for (unsigned i = 0; i < m_mergeInputs.size(); i++) {
        FloatRect destRect = calculateDrawingRect(m_mergeInputs[i]->scaledSubRegion());
        filterContext->drawImage(m_mergeInputs[i]->resultImage()->image(), DeviceColorSpace, destRect);
    }
}

void FEMerge::dump()
{
}

TextStream& FEMerge::externalRepresentation(TextStream& ts) const
{
    ts << "[type=MERGE] ";
    FilterEffect::externalRepresentation(ts);
    ts << "[merge inputs=[";
    unsigned x = 0;
    unsigned size = m_mergeInputs.size();
    while (x < size) {
        ts << m_mergeInputs[x];
        x++;
        if (x < m_mergeInputs.size())
            ts << ", ";
    }
    ts << "]]";
    return ts;
}

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)
