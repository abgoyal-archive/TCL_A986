

#include "config.h"

#if ENABLE(MATHML)

#include "MathMLTextElement.h"

#include "MathMLNames.h"
#include "RenderObject.h"

namespace WebCore {
    
using namespace MathMLNames;

MathMLTextElement::MathMLTextElement(const QualifiedName& tagName, Document* document)
    : MathMLElement(tagName, document)
{
}

PassRefPtr<MathMLTextElement> MathMLTextElement::create(const QualifiedName& tagName, Document* document)
{
    return new MathMLTextElement(tagName, document);
}

RenderObject* MathMLTextElement::createRenderer(RenderArena* , RenderStyle* style)
{
    return RenderObject::createObject(this, style);
}

    
}

#endif // ENABLE(MATHML)

