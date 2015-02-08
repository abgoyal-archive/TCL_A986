

#ifndef WMLOptionElement_h
#define WMLOptionElement_h

#if ENABLE(WML)
#include "OptionElement.h"
#include "WMLFormControlElement.h"
#include "WMLEventHandlingElement.h"

namespace WebCore {

class WMLOptionElement : public WMLFormControlElement, public WMLEventHandlingElement, public OptionElement {
public:
    WMLOptionElement(const QualifiedName& tagName, Document*);
    virtual ~WMLOptionElement();

    virtual const AtomicString& formControlType() const;

    virtual bool rendererIsNeeded(RenderStyle*) { return false; }

    virtual void accessKeyAction(bool sendToAnyElement);
    virtual void childrenChanged(bool changedByParser = false, Node* beforeChange = 0, Node* afterChange = 0, int childCountDelta = 0);
    virtual void parseMappedAttribute(MappedAttribute*);

    virtual void attach();
    virtual void detach();
    virtual void setRenderStyle(PassRefPtr<RenderStyle>);

    virtual void insertedIntoDocument();

    virtual bool selected() const;
    virtual void setSelectedState(bool);

    virtual String text() const;
    virtual String textIndentedToRespectGroupLabel() const;
    virtual String value() const;

private:
    virtual RenderStyle* nonRendererRenderStyle() const;
    void handleIntrinsicEventIfNeeded();

private:
    OptionElementData m_data;
    RefPtr<RenderStyle> m_style;
};

}

#endif
#endif
