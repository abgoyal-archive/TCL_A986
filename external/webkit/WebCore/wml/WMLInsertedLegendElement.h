

#ifndef WMLInsertedLegendElement_h
#define WMLInsertedLegendElement_h

#if ENABLE(WML)
#include "WMLElement.h"

namespace WebCore {

class WMLInsertedLegendElement : public WMLElement {
public:
    WMLInsertedLegendElement(const QualifiedName& tagName, Document*);
    virtual ~WMLInsertedLegendElement();
};

}

#endif
#endif
