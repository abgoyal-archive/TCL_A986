

#ifndef HTMLHeadingElement_h
#define HTMLHeadingElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLHeadingElement : public HTMLElement {
public:
    HTMLHeadingElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 5; }
    virtual bool checkDTD(const Node* newChild);

    String align() const;
    void setAlign(const String&);
};

} // namespace WebCore

#endif // HTMLHeadingElement_h
