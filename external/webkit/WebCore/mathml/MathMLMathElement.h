

#ifndef MathMLMathElement_h
#define MathMLMathElement_h

#if ENABLE(MATHML)
#include "MathMLInlineContainerElement.h"

namespace WebCore {
    
class MathMLMathElement : public MathMLInlineContainerElement {
public:
    static PassRefPtr<MathMLMathElement> create(const QualifiedName& tagName, Document*);
    
protected:
    MathMLMathElement(const QualifiedName& tagName, Document*);
    
};
    
}

#endif // ENABLE(MATHML)
#endif // MathMLMathElement_h
