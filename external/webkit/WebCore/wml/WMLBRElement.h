

#ifndef WMLBRElement_h
#define WMLBRElement_h

#if ENABLE(WML)
#include "WMLElement.h"

namespace WebCore {

class WMLBRElement : public WMLElement {
public:
    WMLBRElement(const QualifiedName& tagName, Document*);

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
};

}

#endif
#endif
