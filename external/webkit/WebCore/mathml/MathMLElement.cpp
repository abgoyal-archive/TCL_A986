

#include "config.h"

#if ENABLE(MATHML)

#include "MathMLElement.h"

#include "MathMLNames.h"
#include "RenderObject.h"

namespace WebCore {
    
using namespace MathMLNames;
    
MathMLElement::MathMLElement(const QualifiedName& tagName, Document* document)
    : StyledElement(tagName, document, CreateElementZeroRefCount)
{
}
    
PassRefPtr<MathMLElement> MathMLElement::create(const QualifiedName& tagName, Document* document)
{
    return new MathMLElement(tagName, document);
}

RenderObject* MathMLElement::createRenderer(RenderArena*, RenderStyle* style)
{
    return RenderObject::createObject(this, style);
}
    
    
}

#endif // ENABLE(MATHML)
