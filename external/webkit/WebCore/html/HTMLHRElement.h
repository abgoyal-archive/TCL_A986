

#ifndef HTMLHRElement_h
#define HTMLHRElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLHRElement : public HTMLElement {
public:
    HTMLHRElement(const QualifiedName&, Document*);
    ~HTMLHRElement();
    
    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }
    
    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    String align() const;
    void setAlign(const String&);

    bool noShade() const;
    void setNoShade(bool);

    String size() const;
    void setSize(const String&);

    String width() const;
    void setWidth(const String&);
};

} // namespace WebCore

#endif // HTMLHRElement_h
