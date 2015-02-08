

#ifndef MathMLTextElement_h
#define MathMLTextElement_h

#if ENABLE(MATHML)
#include "MathMLElement.h"

namespace WebCore {
    
class MathMLTextElement : public MathMLElement {
public:
    static PassRefPtr<MathMLTextElement> create(const QualifiedName& tagName, Document*);

    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
    
protected:
    MathMLTextElement(const QualifiedName& tagName, Document*);
    
};
    
}

#endif // ENABLE(MATHML)
#endif // MathMLTextElement_h
