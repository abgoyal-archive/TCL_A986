

#ifndef HTMLBaseFontElement_h
#define HTMLBaseFontElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLBaseFontElement : public HTMLElement {
public:
    HTMLBaseFontElement(const QualifiedName&, Document*);

    int size() const;
    void setSize(int);

private:
    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }
};

} // namespace

#endif
