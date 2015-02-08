

#ifndef WMLFormControlElement_h
#define WMLFormControlElement_h

#if ENABLE(WML)
#include "WMLElement.h"

namespace WebCore {

class WMLFormControlElement : public WMLElement {
public:
    WMLFormControlElement(const QualifiedName&, Document*);
    virtual ~WMLFormControlElement();

    virtual bool isFormControlElement() const { return true; }
    virtual bool isReadOnlyFormControl() const { return false; }
    virtual bool isTextFormControl() const { return false; }

    virtual bool formControlValueMatchesRenderer() const { return m_valueMatchesRenderer; }
    virtual void setFormControlValueMatchesRenderer(bool b = true) { m_valueMatchesRenderer = b; }

    virtual bool supportsFocus() const;
    virtual bool isFocusable() const;

    virtual void attach();
    virtual void recalcStyle(StyleChange);

private:
    bool m_valueMatchesRenderer;
};

}

#endif
#endif
