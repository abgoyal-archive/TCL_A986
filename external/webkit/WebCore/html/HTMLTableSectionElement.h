

#ifndef HTMLTableSectionElement_h
#define HTMLTableSectionElement_h

#include "HTMLTablePartElement.h"

namespace WebCore {

class HTMLTableSectionElement : public HTMLTablePartElement {
public:
    HTMLTableSectionElement(const QualifiedName& tagName, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusOptional; }
    virtual int tagPriority() const { return 8; }
    virtual bool checkDTD(const Node*);
    virtual ContainerNode* addChild(PassRefPtr<Node>);
    virtual bool canHaveAdditionalAttributeStyleDecls() const { return true; }
    virtual void additionalAttributeStyleDecls(Vector<CSSMutableStyleDeclaration*>&);

    PassRefPtr<HTMLElement> insertRow(int index, ExceptionCode&);
    void deleteRow(int index, ExceptionCode&);

    int numRows() const;

    String align() const;
    void setAlign(const String&);

    String ch() const;
    void setCh(const String&);

    String chOff() const;
    void setChOff(const String&);

    String vAlign() const;
    void setVAlign(const String&);

    PassRefPtr<HTMLCollection> rows();
};

} //namespace

#endif
