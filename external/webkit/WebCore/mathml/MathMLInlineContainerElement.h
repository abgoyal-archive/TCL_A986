

#ifndef MathMLInlineContainerElement_h
#define MathMLInlineContainerElement_h

#if ENABLE(MATHML)
#include "MathMLElement.h"

namespace WebCore {
    
class MathMLInlineContainerElement : public MathMLElement {
public:
    static PassRefPtr<MathMLInlineContainerElement> create(const QualifiedName& tagName, Document*);

    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);

protected:
    MathMLInlineContainerElement(const QualifiedName& tagName, Document*);

};
    
}

#endif // ENABLE(MATHML)
#endif // MathMLInlineContainerElement_h
