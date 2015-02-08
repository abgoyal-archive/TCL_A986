

#ifndef HTMLParagraphElement_h
#define HTMLParagraphElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLParagraphElement : public HTMLElement {
public:
    HTMLParagraphElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 3; }
    virtual bool checkDTD(const Node* newChild);

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    String align() const;
    void setAlign(const String&);
};

} // namespace WebCore

#endif // HTMLParagraphElement_h
