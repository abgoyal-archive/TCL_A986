

#ifndef WMLAccessElement_h
#define WMLAccessElement_h

#if ENABLE(WML)
#include "WMLElement.h"

namespace WebCore {

class WMLAccessElement : public WMLElement {
public:
    WMLAccessElement(const QualifiedName& tagName, Document*);

    virtual void parseMappedAttribute(MappedAttribute*);
    virtual void insertedIntoDocument();

private:
    String m_domain;
    String m_path;
};

}

#endif
#endif
