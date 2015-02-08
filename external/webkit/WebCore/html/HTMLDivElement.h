

#ifndef HTMLDivElement_h
#define HTMLDivElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLDivElement : public HTMLElement {
public:
    HTMLDivElement(const QualifiedName&, Document*);
    ~HTMLDivElement();

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 5; }

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    String align() const;
    void setAlign(const String&);
};

} // namespace WebCore

#endif // HTMLDivElement_h
