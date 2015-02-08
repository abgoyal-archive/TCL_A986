

#ifndef WMLMetaElement_h
#define WMLMetaElement_h

#if ENABLE(WML)
#include "WMLElement.h"

namespace WebCore {

class WMLMetaElement : public WMLElement {
public:
    WMLMetaElement(const QualifiedName& tagName, Document*);
    virtual ~WMLMetaElement();

    virtual void parseMappedAttribute(MappedAttribute*);
    virtual void insertedIntoDocument();

private:
    String m_equiv;
    String m_content;
};

}

#endif
#endif
