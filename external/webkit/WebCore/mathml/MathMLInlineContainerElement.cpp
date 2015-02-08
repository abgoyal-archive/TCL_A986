

#include "config.h"

#if ENABLE(MATHML)

#include "MathMLInlineContainerElement.h"

#include "MathMLNames.h"
#include "RenderMathMLBlock.h"

namespace WebCore {
    
using namespace MathMLNames;

MathMLInlineContainerElement::MathMLInlineContainerElement(const QualifiedName& tagName, Document* document)
    : MathMLElement(tagName, document)
{
}

PassRefPtr<MathMLInlineContainerElement> MathMLInlineContainerElement::create(const QualifiedName& tagName, Document* document)
{
    return new MathMLInlineContainerElement(tagName, document);
}

RenderObject* MathMLInlineContainerElement::createRenderer(RenderArena *arena, RenderStyle* style)
{
    // FIXME: This method will contain the specialized renderers based on element name
    RenderObject* object = new (arena) RenderMathMLBlock(this);
    object->setStyle(style);
    return object;
}
    
    
}

#endif // ENABLE(MATHML)

