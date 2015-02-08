

#ifndef WMLPrevElement_h
#define WMLPrevElement_h

#if ENABLE(WML)
#include "WMLTaskElement.h"

namespace WebCore {

class WMLPrevElement : public WMLTaskElement {
public:
    WMLPrevElement(const QualifiedName& tagName, Document*);
    virtual ~WMLPrevElement();

    virtual void executeTask();
};

}

#endif
#endif
