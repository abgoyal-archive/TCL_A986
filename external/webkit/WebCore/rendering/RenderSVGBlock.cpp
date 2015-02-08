

#include "config.h"

#if ENABLE(SVG)
#include "RenderSVGBlock.h"

#include "SVGElement.h"

namespace WebCore {

RenderSVGBlock::RenderSVGBlock(SVGElement* node) 
    : RenderBlock(node)
{
}

void RenderSVGBlock::setStyle(PassRefPtr<RenderStyle> style) 
{
    RefPtr<RenderStyle> useStyle = style;

    // SVG text layout code expects us to be a block-level style element.   
    if (useStyle->display() == NONE)
        setChildrenInline(false);
    else if (useStyle->isDisplayInlineType()) {
        RefPtr<RenderStyle> newStyle = RenderStyle::create();
        newStyle->inheritFrom(useStyle.get());
        newStyle->setDisplay(BLOCK);
        useStyle = newStyle.release();
    }

    RenderBlock::setStyle(useStyle.release());
    setReplaced(false);

    //FIXME: Once overflow rules are supported by SVG we should
    //probably map the CSS overflow rules rather than just ignoring
    //them
    setHasOverflowClip(false);
}

}

#endif // ENABLE(SVG)
