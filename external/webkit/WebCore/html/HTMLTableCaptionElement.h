

#ifndef HTMLTableCaptionElement_h
#define HTMLTableCaptionElement_h

#include "HTMLTablePartElement.h"

namespace WebCore {

class HTMLTableCaptionElement : public HTMLTablePartElement {
public:
    HTMLTableCaptionElement(const QualifiedName&, Document*);
    
    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 5; }
    
    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    String align() const;
    void setAlign(const String&);
};

} //namespace

#endif
