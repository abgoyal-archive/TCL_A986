
#ifndef HTMLQuoteElement_h
#define HTMLQuoteElement_h

#include "HTMLElement.h"

namespace WebCore {

class String;

class HTMLQuoteElement : public HTMLElement {
public:
    HTMLQuoteElement(const QualifiedName&, Document*);
    
    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 1; }

    virtual void insertedIntoDocument();

    String cite() const;
    void setCite(const String&);
};

} //namespace

#endif
