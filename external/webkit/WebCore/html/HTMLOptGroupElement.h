

#ifndef HTMLOptGroupElement_h
#define HTMLOptGroupElement_h

#include "HTMLFormControlElement.h"
#include "OptionGroupElement.h"

namespace WebCore {
    
class HTMLSelectElement;

class HTMLOptGroupElement : public HTMLFormControlElement, public OptionGroupElement {
public:
    HTMLOptGroupElement(const QualifiedName&, Document*, HTMLFormElement* = 0);

    virtual bool checkDTD(const Node*);
    virtual const AtomicString& formControlType() const;
    virtual bool supportsFocus() const;
    virtual bool isFocusable() const;
    virtual void parseMappedAttribute(MappedAttribute*);
    virtual bool rendererIsNeeded(RenderStyle*) { return false; }
    virtual void attach();
    virtual void detach();
    virtual void setRenderStyle(PassRefPtr<RenderStyle>);

    virtual bool insertBefore(PassRefPtr<Node> newChild, Node* refChild, ExceptionCode&, bool shouldLazyAttach = false);
    virtual bool replaceChild(PassRefPtr<Node> newChild, Node* oldChild, ExceptionCode&, bool shouldLazyAttach = false);
    virtual bool removeChild(Node* child, ExceptionCode&);
    virtual bool appendChild(PassRefPtr<Node> newChild, ExceptionCode&, bool shouldLazyAttach = false);
    virtual bool removeChildren();
    virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);

    String label() const;
    void setLabel(const String&);
    
    virtual String groupLabelText() const;
    HTMLSelectElement* ownerSelectElement() const;
    virtual void accessKeyAction(bool sendToAnyElement);
    
private:
    virtual RenderStyle* nonRendererRenderStyle() const;
    
    void recalcSelectOptions();

    RefPtr<RenderStyle> m_style;
};

} //namespace

#endif
