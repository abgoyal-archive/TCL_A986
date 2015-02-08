

#ifndef WMLRefreshElement_h
#define WMLRefreshElement_h

#if ENABLE(WML)
#include "WMLTaskElement.h"

namespace WebCore {

class WMLRefreshElement : public WMLTaskElement {
public:
    WMLRefreshElement(const QualifiedName& tagName, Document*);
    virtual ~WMLRefreshElement();

    virtual void executeTask();
};

}

#endif
#endif
