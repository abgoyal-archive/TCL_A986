

#ifndef HTMLTablePartElement_h
#define HTMLTablePartElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLTablePartElement : public HTMLElement {
public:
    HTMLTablePartElement(const QualifiedName& tagName, Document* doc)
        : HTMLElement(tagName, doc)
        { }

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);
};

} //namespace

#endif
