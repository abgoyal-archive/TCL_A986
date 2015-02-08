

#ifndef WMLTemplateElement_h
#define WMLTemplateElement_h

#if ENABLE(WML)
#include "WMLElement.h"
#include "WMLEventHandlingElement.h"

namespace WebCore {

class WMLTemplateElement : public WMLElement, public WMLEventHandlingElement {
public:
    WMLTemplateElement(const QualifiedName&, Document*);
    virtual ~WMLTemplateElement();

    virtual void parseMappedAttribute(MappedAttribute*);

    static void registerTemplatesInDocument(Document*);
};

}

#endif
#endif
