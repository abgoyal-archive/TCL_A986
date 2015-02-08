
#ifndef HTMLTitleElement_h
#define HTMLTitleElement_h

#include "HTMLElement.h"

namespace WebCore {

class HTMLTitleElement : public HTMLElement {
public:
    HTMLTitleElement(const QualifiedName&, Document*);
    ~HTMLTitleElement();

    virtual bool checkDTD(const Node* newChild) { return newChild->isTextNode(); }

    virtual void insertedIntoDocument();
    virtual void removedFromDocument();
    virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);

    String text() const;
    void setText(const String&);

protected:
    String m_title;
};

} //namespace

#endif
