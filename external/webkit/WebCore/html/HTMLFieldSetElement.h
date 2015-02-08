

#ifndef HTMLFieldSetElement_h
#define HTMLFieldSetElement_h

#include "HTMLFormControlElement.h"

namespace WebCore {
    class RenderStyle;
}

namespace WebCore {

class HTMLFormElement;
class Document;
class Node;

class HTMLFieldSetElement : public HTMLFormControlElement {
public:
    HTMLFieldSetElement(const QualifiedName&, Document*, HTMLFormElement* = 0);
    virtual ~HTMLFieldSetElement();
    
    virtual int tagPriority() const { return 3; }
    virtual bool checkDTD(const Node* newChild);

    virtual bool supportsFocus() const;
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
    virtual const AtomicString& formControlType() const;

    virtual bool willValidate() const { return false; }
};

} //namespace

#endif
