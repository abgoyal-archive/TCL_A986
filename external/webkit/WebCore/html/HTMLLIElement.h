

#ifndef HTMLLIElement_h
#define HTMLLIElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLLIElement : public HTMLElement {
public:
    HTMLLIElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusOptional; }
    virtual int tagPriority() const { return 3; }

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    virtual void attach();

    String type() const;
    void setType(const String&);

    int value() const;
    void setValue(int);

private:
    int m_requestedValue;
};

} //namespace

#endif
