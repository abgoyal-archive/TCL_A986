

#include "config.h"
#include "RenderScrollbarTheme.h"
#include "RenderScrollbar.h"
#include <wtf/StdLibExtras.h>

namespace WebCore {

RenderScrollbarTheme* RenderScrollbarTheme::renderScrollbarTheme()
{
    DEFINE_STATIC_LOCAL(RenderScrollbarTheme, theme, ());
    return &theme;
}

void RenderScrollbarTheme::buttonSizesAlongTrackAxis(Scrollbar* scrollbar, int& beforeSize, int& afterSize)
{
    IntRect firstButton = backButtonRect(scrollbar, BackButtonStartPart);
    IntRect secondButton = forwardButtonRect(scrollbar, ForwardButtonStartPart);
    IntRect thirdButton = backButtonRect(scrollbar, BackButtonEndPart);
    IntRect fourthButton = forwardButtonRect(scrollbar, ForwardButtonEndPart);
    if (scrollbar->orientation() == HorizontalScrollbar) {
        beforeSize = firstButton.width() + secondButton.width();
        afterSize = thirdButton.width() + fourthButton.width();
    } else {
        beforeSize = firstButton.height() + secondButton.height();
        afterSize = thirdButton.height() + fourthButton.height();
    }
}

bool RenderScrollbarTheme::hasButtons(Scrollbar* scrollbar)
{
    int startSize;
    int endSize;
    buttonSizesAlongTrackAxis(scrollbar, startSize, endSize);
    return (startSize + endSize) <= (scrollbar->orientation() == HorizontalScrollbar ? scrollbar->width() : scrollbar->height());
}

bool RenderScrollbarTheme::hasThumb(Scrollbar* scrollbar)
{
    return trackLength(scrollbar) - thumbLength(scrollbar) >= 0;
}

int RenderScrollbarTheme::minimumThumbLength(Scrollbar* scrollbar)
{
    return toRenderScrollbar(scrollbar)->minimumThumbLength();
}

IntRect RenderScrollbarTheme::backButtonRect(Scrollbar* scrollbar, ScrollbarPart partType, bool)
{
    return toRenderScrollbar(scrollbar)->buttonRect(partType);
}

IntRect RenderScrollbarTheme::forwardButtonRect(Scrollbar* scrollbar, ScrollbarPart partType, bool)
{
    return toRenderScrollbar(scrollbar)->buttonRect(partType);
}

IntRect RenderScrollbarTheme::trackRect(Scrollbar* scrollbar, bool)
{
    if (!hasButtons(scrollbar))
        return scrollbar->frameRect();
    
    int startLength;
    int endLength;
    buttonSizesAlongTrackAxis(scrollbar, startLength, endLength);
    
    return toRenderScrollbar(scrollbar)->trackRect(startLength, endLength);
}

IntRect RenderScrollbarTheme::constrainTrackRectToTrackPieces(Scrollbar* scrollbar, const IntRect& rect)
{ 
    IntRect backRect = toRenderScrollbar(scrollbar)->trackPieceRectWithMargins(BackTrackPart, rect);
    IntRect forwardRect = toRenderScrollbar(scrollbar)->trackPieceRectWithMargins(ForwardTrackPart, rect);
    IntRect result = rect;
    if (scrollbar->orientation() == HorizontalScrollbar) {
        result.setX(backRect.x());
        result.setWidth(forwardRect.right() - backRect.x());
    } else {
        result.setY(backRect.y());
        result.setHeight(forwardRect.bottom() - backRect.y());
    }
    return result;
}

void RenderScrollbarTheme::paintScrollCorner(ScrollView*, GraphicsContext* context, const IntRect& cornerRect)
{
    // FIXME: Implement.
    context->fillRect(cornerRect, Color::white, DeviceColorSpace);
}

void RenderScrollbarTheme::paintScrollbarBackground(GraphicsContext* context, Scrollbar* scrollbar)
{
    toRenderScrollbar(scrollbar)->paintPart(context, ScrollbarBGPart, scrollbar->frameRect());
}

void RenderScrollbarTheme::paintTrackBackground(GraphicsContext* context, Scrollbar* scrollbar, const IntRect& rect)
{
    toRenderScrollbar(scrollbar)->paintPart(context, TrackBGPart, rect);
}

void RenderScrollbarTheme::paintTrackPiece(GraphicsContext* context, Scrollbar* scrollbar, const IntRect& rect, ScrollbarPart part)
{
    toRenderScrollbar(scrollbar)->paintPart(context, part, rect);
}

void RenderScrollbarTheme::paintButton(GraphicsContext* context, Scrollbar* scrollbar, const IntRect& rect, ScrollbarPart part)
{
    toRenderScrollbar(scrollbar)->paintPart(context, part, rect);
}

void RenderScrollbarTheme::paintThumb(GraphicsContext* context, Scrollbar* scrollbar, const IntRect& rect)
{
    toRenderScrollbar(scrollbar)->paintPart(context, ThumbPart, rect);
}

}
