

#ifndef SVGFilterBuilder_h
#define SVGFilterBuilder_h

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "AtomicStringHash.h"
#include "FilterEffect.h"
#include "PlatformString.h"

#include <wtf/HashMap.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {
    
    class SVGFilterBuilder : public RefCounted<SVGFilterBuilder> {
    public:
        SVGFilterBuilder();

        void add(const AtomicString& id, RefPtr<FilterEffect> effect);

        FilterEffect* getEffectById(const AtomicString& id) const;
        FilterEffect* lastEffect() const { return m_lastEffect.get(); }

        void clearEffects();

    private:
        HashMap<AtomicString, RefPtr<FilterEffect> > m_builtinEffects;
        HashMap<AtomicString, RefPtr<FilterEffect> > m_namedEffects;

        RefPtr<FilterEffect> m_lastEffect;
    };
    
} //namespace WebCore

#endif // ENABLE(SVG) && ENABLE(FILTERS)
#endif
