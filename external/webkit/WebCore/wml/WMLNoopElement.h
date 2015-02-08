

#ifndef WMLNoopElement_h
#define WMLNoopElement_h

#if ENABLE(WML)
#include "WMLElement.h"

namespace WebCore {

class WMLNoopElement : public WMLElement {
public:
    WMLNoopElement(const QualifiedName& tagName, Document*);
    virtual ~WMLNoopElement();

    virtual void insertedIntoDocument();
};

}

#endif
#endif
