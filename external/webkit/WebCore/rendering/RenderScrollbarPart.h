

#ifndef RenderScrollbarPart_h
#define RenderScrollbarPart_h

#include "RenderBlock.h"
#include "ScrollTypes.h"

namespace WebCore {

class RenderScrollbar;

class RenderScrollbarPart : public RenderBlock {
public:
    RenderScrollbarPart(Node*, RenderScrollbar* = 0, ScrollbarPart = NoPart);
    virtual ~RenderScrollbarPart();

    virtual const char* renderName() const { return "RenderScrollbarPart"; }
    
    virtual bool requiresLayer() const { return false; }

    virtual void layout();
    virtual void calcPrefWidths();
    
    void paintIntoRect(GraphicsContext*, int tx, int ty, const IntRect&);
    
protected:
    virtual void styleWillChange(StyleDifference diff, const RenderStyle* newStyle);
    virtual void styleDidChange(StyleDifference, const RenderStyle* oldStyle);
    virtual void imageChanged(WrappedImagePtr, const IntRect* = 0);

private:
    void layoutHorizontalPart();
    void layoutVerticalPart();

    void computeScrollbarWidth();
    void computeScrollbarHeight();
    
    RenderScrollbar* m_scrollbar;
    ScrollbarPart m_part;
};

} // namespace WebCore

#endif // RenderScrollbarPart_h
