

#ifndef WMLTaskElement_h
#define WMLTaskElement_h

#if ENABLE(WML)
#include "WMLElement.h"

#include <wtf/Vector.h>

namespace WebCore {

class WMLPageState;
class WMLSetvarElement;

class WMLTaskElement : public WMLElement {
public:
    WMLTaskElement(const QualifiedName& tagName, Document*);
    virtual ~WMLTaskElement();

    virtual bool isWMLTaskElement() const { return true; }

    virtual void insertedIntoDocument();
    virtual void removedFromDocument();
    virtual void executeTask() = 0;

    void registerVariableSetter(WMLSetvarElement*);
    void deregisterVariableSetter(WMLSetvarElement*);

protected:
    void storeVariableState(WMLPageState*);

private:
    Vector<WMLSetvarElement*> m_variableSetterElements;
};

}

#endif
#endif
