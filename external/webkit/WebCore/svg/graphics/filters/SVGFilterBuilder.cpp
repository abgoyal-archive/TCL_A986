


#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFilterBuilder.h"

#include "FilterEffect.h"
#include "PlatformString.h"
#include "SourceAlpha.h"
#include "SourceGraphic.h"

#include <wtf/HashMap.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

SVGFilterBuilder::SVGFilterBuilder()
{
    m_builtinEffects.add(SourceGraphic::effectName(), SourceGraphic::create());
    m_builtinEffects.add(SourceAlpha::effectName(), SourceAlpha::create());
}

void SVGFilterBuilder::add(const AtomicString& id, RefPtr<FilterEffect> effect)
{
    if (id.isEmpty()) {
        m_lastEffect = effect;
        return;
    }

    if (m_builtinEffects.contains(id))
        return;

    m_lastEffect = effect;
    m_namedEffects.set(id, m_lastEffect);
}

FilterEffect* SVGFilterBuilder::getEffectById(const AtomicString& id) const
{
    if (id.isEmpty()) {
        if (m_lastEffect)
            return m_lastEffect.get();

        return m_builtinEffects.get(SourceGraphic::effectName()).get();
    }

    if (m_builtinEffects.contains(id))
        return m_builtinEffects.get(id).get();

    return m_namedEffects.get(id).get();
}

void SVGFilterBuilder::clearEffects()
{
    m_lastEffect = 0;
    m_namedEffects.clear();
}

} // namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)
