

#ifndef WMLFieldSetElement_h
#define WMLFieldSetElement_h

#if ENABLE(WML)
#include "WMLElement.h"

namespace WebCore {

class WMLFieldSetElement : public WMLElement {
public:
    WMLFieldSetElement(const QualifiedName& tagName, Document*);
    virtual ~WMLFieldSetElement();

    virtual void insertedIntoDocument();
    virtual void removedFromDocument();

    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);

private:
    RefPtr<WMLElement> m_insertedLegendElement;
};

}

#endif
#endif
