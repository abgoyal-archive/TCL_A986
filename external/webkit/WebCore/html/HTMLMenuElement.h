

#ifndef HTMLMenuElement_h
#define HTMLMenuElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLMenuElement : public HTMLElement {
public:
    HTMLMenuElement(const QualifiedName&, Document*);
    
    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 5; }

    bool compact() const;
    void setCompact(bool);
};

} //namespace

#endif
