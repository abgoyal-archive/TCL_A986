
#ifndef HTMLFontElement_h
#define HTMLFontElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLFontElement : public HTMLElement {
public:
    HTMLFontElement(const QualifiedName&, Document*);
    
    virtual HTMLTagStatus endTagRequirement() const { return TagStatusRequired; }
    virtual int tagPriority() const { return 1; }

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    String color() const;
    void setColor(const String&);

    String face() const;
    void setFace(const String&);

    String size() const;
    void setSize(const String&);
    
    static bool cssValueFromFontSizeNumber(const String&, int&);
};

} //namespace

#endif
