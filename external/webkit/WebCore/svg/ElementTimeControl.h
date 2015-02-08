

#ifndef ElementTimeControl_h
#define ElementTimeControl_h

#if ENABLE(SVG)

#include "ExceptionCode.h"

namespace WebCore {

    class ElementTimeControl {
    public:
        virtual ~ElementTimeControl() {}
        virtual void beginElement() = 0;
        virtual void beginElementAt(float offset) = 0;
        virtual void endElement() = 0;
        virtual void endElementAt(float offset) = 0;
    };
        
}

#endif

#endif
