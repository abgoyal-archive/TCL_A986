

#ifndef HTMLNoScriptElement_h
#define HTMLNoScriptElement_h

#if ENABLE(XHTMLMP)
#include "HTMLElement.h"

namespace WebCore {

class HTMLNoScriptElement : public HTMLElement {
public:
    HTMLNoScriptElement(const QualifiedName&, Document*);

private:
    virtual ~HTMLNoScriptElement();

    virtual bool checkDTD(const Node*);
    virtual void attach();
    virtual void recalcStyle(StyleChange);
    virtual bool childShouldCreateRenderer(Node*) const;
    virtual bool rendererIsNeeded(RenderStyle*) {  return true; }
};

} //namespace

#endif
#endif
