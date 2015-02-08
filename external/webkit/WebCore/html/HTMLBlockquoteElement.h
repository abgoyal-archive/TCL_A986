

#ifndef HTMLBlockquoteElement_h
#define HTMLBlockquoteElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLBlockquoteElement : public HTMLElement {
public:
    HTMLBlockquoteElement(const QualifiedName&, Document*);

private:
    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 5; }
};

} // namespace WebCore

#endif // HTMLBlockquoteElement_h
