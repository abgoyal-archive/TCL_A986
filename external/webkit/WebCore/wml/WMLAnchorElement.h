

#ifndef WMLAnchorElement_h
#define WMLAnchorElement_h

#if ENABLE(WML)
#include "WMLAElement.h"

namespace WebCore {

class WMLTaskElement;

class WMLAnchorElement : public WMLAElement {
public:
    WMLAnchorElement(const QualifiedName& tagName, Document*);
    virtual ~WMLAnchorElement();

    virtual void defaultEventHandler(Event*);

private:
    friend class WMLTaskElement;
    void registerTask(WMLTaskElement*);
    void deregisterTask(WMLTaskElement*);

    WMLTaskElement* m_task;
};

}

#endif
#endif
