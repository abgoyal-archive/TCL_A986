

#ifndef WMLPElement_h
#define WMLPElement_h

#if ENABLE(WML)
#include "WMLElement.h"

namespace WebCore {

class WMLPElement : public WMLElement {
public:
    WMLPElement(const QualifiedName& tagName, Document*);

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    virtual void insertedIntoDocument();

private:
    String m_mode;
};

}

#endif
#endif
