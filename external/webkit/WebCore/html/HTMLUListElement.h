

#ifndef HTMLUListElement_h
#define HTMLUListElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLUListElement : public HTMLElement {
public:
    HTMLUListElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 5; }

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    bool compact() const;
    void setCompact(bool);

    String type() const;
    void setType(const String&);
};

} //namespace

#endif
