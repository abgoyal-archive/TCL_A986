

#ifndef SourceGraphic_h
#define SourceGrahpic_h

#if ENABLE(FILTERS)
#include "FilterEffect.h"

#include "Filter.h"
#include "PlatformString.h"

namespace WebCore {

    class SourceGraphic : public FilterEffect {
    public:        
        static PassRefPtr<SourceGraphic> create();

        static const AtomicString& effectName();

        virtual bool isSourceInput() { return true; }
        virtual FloatRect calculateEffectRect(Filter*);
        void apply(Filter*);
        void dump();
    
    private:
        SourceGraphic() { }
    };
} //namespace WebCore

#endif // ENABLE(FILTERS)

#endif // SourceGraphic_h
