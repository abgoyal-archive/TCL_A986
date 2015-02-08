

#ifndef WMLAElement_h
#define WMLAElement_h

#if ENABLE(WML)
#include "WMLElement.h"

namespace WebCore {

class WMLAElement : public WMLElement {
public:
    WMLAElement(const QualifiedName& tagName, Document*);

    virtual bool supportsFocus() const;
    virtual bool isMouseFocusable() const;
    virtual bool isKeyboardFocusable(KeyboardEvent*) const;

    virtual void parseMappedAttribute(MappedAttribute*);
    virtual void defaultEventHandler(Event*);

    virtual void accessKeyAction(bool fullAction);
    virtual bool isURLAttribute(Attribute*) const;

    virtual String target() const;
};

}

#endif
#endif
