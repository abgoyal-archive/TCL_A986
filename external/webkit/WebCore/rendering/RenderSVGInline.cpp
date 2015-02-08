

#include "config.h"

#if ENABLE(SVG)
#include "RenderSVGInline.h"

#include "FloatQuad.h"
#include "RenderBlock.h"
#include "SVGInlineFlowBox.h"
#include "SVGInlineTextBox.h"
#include "SVGRootInlineBox.h"

namespace WebCore {
    
RenderSVGInline::RenderSVGInline(Node* n)
    : RenderInline(n)
{
}

InlineFlowBox* RenderSVGInline::createInlineFlowBox()
{
    InlineFlowBox* box = new (renderArena()) SVGInlineFlowBox(this);
    box->setHasVirtualHeight();
    return box;
}

void RenderSVGInline::absoluteRects(Vector<IntRect>& rects, int, int)
{
    InlineRunBox* firstBox = firstLineBox();

    SVGRootInlineBox* rootBox = firstBox ? static_cast<SVGInlineTextBox*>(firstBox)->svgRootInlineBox() : 0;
    RenderBox* object = rootBox ? rootBox->block() : 0;

    if (!object)
        return;

    int xRef = object->x();
    int yRef = object->y();

    for (InlineRunBox* curr = firstBox; curr; curr = curr->nextLineBox()) {
        FloatRect rect(xRef + curr->x(), yRef + curr->y(), curr->width(), curr->height());
        rects.append(enclosingIntRect(localToAbsoluteQuad(rect).boundingBox()));
    }
}

void RenderSVGInline::absoluteQuads(Vector<FloatQuad>& quads)
{
    InlineRunBox* firstBox = firstLineBox();

    SVGRootInlineBox* rootBox = firstBox ? static_cast<SVGInlineTextBox*>(firstBox)->svgRootInlineBox() : 0;
    RenderBox* object = rootBox ? rootBox->block() : 0;

    if (!object)
        return;

    int xRef = object->x();
    int yRef = object->y();

    for (InlineRunBox* curr = firstBox; curr; curr = curr->nextLineBox()) {
        FloatRect rect(xRef + curr->x(), yRef + curr->y(), curr->width(), curr->height());
        quads.append(localToAbsoluteQuad(rect));
    }
}

}

#endif // ENABLE(SVG)
