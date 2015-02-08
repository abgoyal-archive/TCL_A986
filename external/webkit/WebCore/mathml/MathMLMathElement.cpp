

#include "config.h"

#if ENABLE(MATHML)

#include "MathMLMathElement.h"

#include "MathMLNames.h"
#include "RenderObject.h"

namespace WebCore {
    
using namespace MathMLNames;

MathMLMathElement::MathMLMathElement(const QualifiedName& tagName, Document* document)
    : MathMLInlineContainerElement(tagName, document)
{
}

PassRefPtr<MathMLMathElement> MathMLMathElement::create(const QualifiedName& tagName, Document* document)
{
    return new MathMLMathElement(tagName, document);
}

}

#endif // ENABLE(MATHML)
