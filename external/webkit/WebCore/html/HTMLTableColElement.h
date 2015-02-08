

#ifndef HTMLTableColElement_h
#define HTMLTableColElement_h

#include "HTMLTablePartElement.h"

namespace WebCore {

class HTMLTableElement;

class HTMLTableColElement : public HTMLTablePartElement {
public:
    HTMLTableColElement(const QualifiedName& tagName, Document*);

    virtual HTMLTagStatus endTagRequirement() const;
    virtual int tagPriority() const;
    virtual bool checkDTD(const Node*);

    // overrides
    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);
    virtual bool canHaveAdditionalAttributeStyleDecls() const { return true; }
    virtual void additionalAttributeStyleDecls(Vector<CSSMutableStyleDeclaration*>&);
   
    int span() const { return _span; }

    String align() const;
    void setAlign(const String&);

    String ch() const;
    void setCh(const String&);

    String chOff() const;
    void setChOff(const String&);

    void setSpan(int);

    String vAlign() const;
    void setVAlign(const String&);

    String width() const;
    void setWidth(const String&);

protected:
    int _span;
};

} //namespace

#endif
