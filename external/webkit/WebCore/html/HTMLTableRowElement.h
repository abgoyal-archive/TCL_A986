

#ifndef HTMLTableRowElement_h
#define HTMLTableRowElement_h

#include "HTMLTablePartElement.h"

namespace WebCore {

class HTMLTableRowElement : public HTMLTablePartElement {
public:
    HTMLTableRowElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusOptional; }
    virtual int tagPriority() const { return 7; }
    virtual bool checkDTD(const Node*);
    virtual ContainerNode* addChild(PassRefPtr<Node>);

    int rowIndex() const;
    void setRowIndex(int);

    int sectionRowIndex() const;
    void setSectionRowIndex(int);

    PassRefPtr<HTMLElement> insertCell(int index, ExceptionCode&);
    void deleteCell(int index, ExceptionCode&);

    PassRefPtr<HTMLCollection> cells();
    void setCells(HTMLCollection *, ExceptionCode&);

    String align() const;
    void setAlign(const String&);

    String bgColor() const;
    void setBgColor(const String&);

    String ch() const;
    void setCh(const String&);

    String chOff() const;
    void setChOff(const String&);

    String vAlign() const;
    void setVAlign(const String&);
};

} // namespace

#endif
