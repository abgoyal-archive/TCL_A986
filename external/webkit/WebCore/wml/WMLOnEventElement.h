

#ifndef WMLOnEventElement_h
#define WMLOnEventElement_h

#if ENABLE(WML)
#include "WMLElement.h"
#include "WMLIntrinsicEventHandler.h"

namespace WebCore {

class WMLTaskElement;

class WMLOnEventElement : public WMLElement {
public:
    WMLOnEventElement(const QualifiedName& tagName, Document*);

    virtual void parseMappedAttribute(MappedAttribute*);

    void registerTask(WMLTaskElement*);
    void deregisterTask(WMLTaskElement*);

private:
    WMLIntrinsicEventType m_type;
};

}

#endif
#endif
