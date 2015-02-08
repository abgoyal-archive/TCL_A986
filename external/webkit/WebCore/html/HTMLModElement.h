
#ifndef HTMLModElement_h
#define HTMLModElement_h

#include "HTMLElement.h"

namespace WebCore {

class String;

class HTMLModElement : public HTMLElement {
public:
    HTMLModElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 1; }

    String cite() const;
    void setCite(const String&);

    String dateTime() const;
    void setDateTime(const String&);
};

} //namespace

#endif
