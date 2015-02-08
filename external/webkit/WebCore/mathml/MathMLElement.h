

#ifndef MathMLElement_h
#define MathMLElement_h

#if ENABLE(MATHML)
#include "StyledElement.h"

namespace WebCore {
    
class MathMLElement : public StyledElement {
public:
    static PassRefPtr<MathMLElement> create(const QualifiedName& tagName, Document*);
    
    virtual bool isMathMLElement() const { return true; }
    
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
    
protected:
    MathMLElement(const QualifiedName& tagName, Document*);
    
};
    
}

#endif // ENABLE(MATHML)
#endif // MathMLElement_h
