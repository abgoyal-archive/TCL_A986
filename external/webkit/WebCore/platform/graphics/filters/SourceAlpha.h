

#ifndef SourceAlpha_h
#define SourceAlpha_h

#if ENABLE(FILTERS)
#include "FilterEffect.h"

#include "PlatformString.h"
#include "Filter.h"

namespace WebCore {

    class SourceAlpha : public FilterEffect {
    public:        
        static PassRefPtr<SourceAlpha> create();

        static const AtomicString& effectName();

        virtual bool isSourceInput() { return true; }
        virtual FloatRect calculateEffectRect(Filter*);
        void apply(Filter*);
        void dump();
    
    private:
        SourceAlpha() { }
    };
} //namespace WebCore

#endif // ENABLE(FILTERS)

#endif // SourceAlpha_h
