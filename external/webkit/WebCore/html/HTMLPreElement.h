

#ifndef HTMLPreElement_h
#define HTMLPreElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLPreElement : public HTMLElement {
public:
    HTMLPreElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 5; }

    bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    void parseMappedAttribute(MappedAttribute*);

    int width() const;
    void setWidth(int w);

    bool wrap() const;
    void setWrap(bool b);
};

} // namespace WebCore

#endif // HTMLPreElement_h
