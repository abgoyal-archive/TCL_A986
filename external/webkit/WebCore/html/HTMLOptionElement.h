
#ifndef HTMLOptionElement_h
#define HTMLOptionElement_h

#include "HTMLFormControlElement.h"
#include "OptionElement.h"

namespace WebCore {

class HTMLSelectElement;
class HTMLFormElement;
class MappedAttribute;

class HTMLOptionElement : public HTMLFormControlElement, public OptionElement {
    friend class HTMLSelectElement;
    friend class RenderMenuList;

public:
    HTMLOptionElement(const QualifiedName&, Document*, HTMLFormElement* = 0);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusOptional; }
    virtual int tagPriority() const { return 2; }
    virtual bool checkDTD(const Node* newChild);
    virtual bool supportsFocus() const;
    virtual bool isFocusable() const;
    virtual bool rendererIsNeeded(RenderStyle*) { return false; }
    virtual void attach();
    virtual void detach();
    virtual void setRenderStyle(PassRefPtr<RenderStyle>);
    
    virtual const AtomicString& formControlType() const;

    virtual String text() const;
    void setText(const String&, ExceptionCode&);

    int index() const;
    virtual void parseMappedAttribute(MappedAttribute*);

    virtual String value() const;
    void setValue(const String&);

    virtual bool selected() const;
    void setSelected(bool);
    virtual void setSelectedState(bool);

    HTMLSelectElement* ownerSelectElement() const;

    virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);

    bool defaultSelected() const;
    void setDefaultSelected(bool);

    String label() const;
    void setLabel(const String&);

    virtual String textIndentedToRespectGroupLabel() const;

    bool ownElementDisabled() const { return HTMLFormControlElement::disabled(); }
    virtual bool disabled() const;
    
    virtual void insertedIntoTree(bool);
    virtual void accessKeyAction(bool);
    
private:
    virtual RenderStyle* nonRendererRenderStyle() const;

    OptionElementData m_data;
    RefPtr<RenderStyle> m_style;
};

} //namespace

#endif
