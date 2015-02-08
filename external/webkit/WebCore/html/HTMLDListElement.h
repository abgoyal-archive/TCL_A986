

#ifndef HTMLDListElement_h
#define HTMLDListElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLDListElement : public HTMLElement {
public:
    HTMLDListElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 5; }

    bool compact() const;
    void setCompact(bool);
};

} //namespace

#endif
