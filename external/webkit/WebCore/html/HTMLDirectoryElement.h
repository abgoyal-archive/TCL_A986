

#ifndef HTMLDirectoryElement_h
#define HTMLDirectoryElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLDirectoryElement : public HTMLElement {
public:
    HTMLDirectoryElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 5; }

    bool compact() const;
    void setCompact(bool);
};

} //namespace

#endif
