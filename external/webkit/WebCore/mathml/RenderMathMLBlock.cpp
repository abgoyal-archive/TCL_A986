

#include "config.h"

#if ENABLE(MATHML)

#include "RenderMathMLBlock.h"

#include "FontSelector.h"
#include "GraphicsContext.h"
#include "MathMLNames.h"
#include "RenderInline.h"
#include "RenderText.h"

namespace WebCore {
    
using namespace MathMLNames;
    
RenderMathMLBlock::RenderMathMLBlock(Node* container) 
    : RenderBlock(container) 
{
}

bool RenderMathMLBlock::isChildAllowed(RenderObject* child, RenderStyle*) const
{
    return child->node() && child->node()->nodeType() == Node::ELEMENT_NODE;
}

PassRefPtr<RenderStyle> RenderMathMLBlock::makeBlockStyle()
{
    RefPtr<RenderStyle> newStyle = RenderStyle::create();
    newStyle->inheritFrom(style());
    newStyle->setDisplay(BLOCK);
    return newStyle;
}

int RenderMathMLBlock::nonOperatorHeight() const
{
    if (!isRenderMathMLOperator())
        return offsetHeight();
        
    return 0;
}

void RenderMathMLBlock::stretchToHeight(int height) 
{
    for (RenderObject* current = firstChild(); current; current = current->nextSibling())
       if (current->isRenderMathMLBlock()) {
          RenderMathMLBlock* block = toRenderMathMLBlock(current);
          block->stretchToHeight(height);
       }
}

#if ENABLE(DEBUG_MATH_LAYOUT)
void RenderMathMLBlock::paint(PaintInfo& info, int tx, int ty)
{
    RenderBlock::paint(info, tx, ty);
    
    if (info.context->paintingDisabled() || info.phase != PaintPhaseForeground)
        return;

    tx += x();
    ty += y();
    
    info.context->save();
    
    info.context->setStrokeThickness(1.0f);
    info.context->setStrokeStyle(SolidStroke);
    info.context->setStrokeColor(Color(0, 0, 255), sRGBColorSpace);
    
    info.context->drawLine(IntPoint(tx, ty), IntPoint(tx + offsetWidth(), ty));
    info.context->drawLine(IntPoint(tx + offsetWidth(), ty), IntPoint(tx + offsetWidth(), ty + offsetHeight()));
    info.context->drawLine(IntPoint(tx, ty + offsetHeight()), IntPoint(tx + offsetWidth(), ty + offsetHeight()));
    info.context->drawLine(IntPoint(tx, ty), IntPoint(tx, ty + offsetHeight()));
    
    int baseline = baselinePosition(true);
    
    info.context->setStrokeColor(Color(255, 0, 0), sRGBColorSpace);
    
    info.context->drawLine(IntPoint(tx, ty + baseline), IntPoint(tx + offsetWidth(), ty + baseline));
    
    info.context->restore();
    
}
#endif // ENABLE(DEBUG_MATH_LAYOUT)


}    

#endif
