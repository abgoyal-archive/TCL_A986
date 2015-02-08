

#ifndef RenderSVGInlineText_h
#define RenderSVGInlineText_h

#if ENABLE(SVG)

#include "RenderText.h"

namespace WebCore {
class RenderSVGInlineText : public RenderText {
public:
    RenderSVGInlineText(Node*, PassRefPtr<StringImpl>);

private:
    virtual const char* renderName() const { return "RenderSVGInlineText"; }
        
    virtual void styleDidChange(StyleDifference, const RenderStyle*);

    virtual void absoluteRects(Vector<IntRect>& rects, int tx, int ty);
    virtual void absoluteQuads(Vector<FloatQuad>&);

    virtual bool requiresLayer() const { return false; }
    virtual IntRect selectionRectForRepaint(RenderBoxModelObject* repaintContainer, bool clipToVisibleContent = true);
    virtual bool isSVGText() const { return true; }

    virtual IntRect localCaretRect(InlineBox*, int caretOffset, int* extraWidthToEndOfLine = 0);
    virtual VisiblePosition positionForPoint(const IntPoint&);

    virtual void destroy();

    virtual InlineTextBox* createTextBox();
    IntRect computeRepaintRectForRange(RenderBoxModelObject* repaintContainer, int startPos, int endPos);
    FloatQuad computeRepaintQuadForRange(RenderBoxModelObject* repaintContainer, int startPos, int endPos);
};

}

#endif // ENABLE(SVG)

#endif // !RenderSVGInlineText_h
