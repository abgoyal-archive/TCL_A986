

#ifndef WMLTableElement_h
#define WMLTableElement_h

#if ENABLE(WML)
#include "WMLElement.h"

namespace WebCore {

class HTMLCollection;

class WMLTableElement : public WMLElement {
public:
    WMLTableElement(const QualifiedName& tagName, Document*);
    virtual ~WMLTableElement();

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    virtual void finishParsingChildren();

private:
    Vector<WMLElement*> scanTableChildElements(WMLElement* startElement, const QualifiedName& tagName) const;

    bool tryMergeAdjacentTextCells(Node* item, Node* nextItem) const;
    void transferAllChildrenOfElementToTargetElement(WMLElement* sourceElement, WMLElement* targetElement, unsigned startOffset) const;
    void joinSuperflousColumns(Vector<WMLElement*>& columnElements, WMLElement* rowElement) const;
    void padWithEmptyColumns(Vector<WMLElement*>& columnElements, WMLElement* rowElement) const;
    void alignCells(Vector<WMLElement*>& columnElements, WMLElement* rowElement) const;

    unsigned m_columns;
    String m_alignment;
};

}

#endif
#endif
