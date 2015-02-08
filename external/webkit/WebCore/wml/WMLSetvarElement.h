

#ifndef WMLSetvarElement_h
#define WMLSetvarElement_h

#if ENABLE(WML)
#include "WMLElement.h"

namespace WebCore {

class WMLSetvarElement : public WMLElement {
public:
    WMLSetvarElement(const QualifiedName& tagName, Document*);
    virtual ~WMLSetvarElement();

    virtual void parseMappedAttribute(MappedAttribute*);
    virtual void insertedIntoDocument();
    virtual void removedFromDocument();

    String name() const;
    String value() const;
};

}

#endif
#endif
