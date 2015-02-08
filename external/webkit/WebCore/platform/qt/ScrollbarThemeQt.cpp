

#include "config.h"
#include "ScrollbarThemeQt.h"

#include "GraphicsContext.h"
#include "PlatformMouseEvent.h"
#include "RenderThemeQt.h"
#include "Scrollbar.h"
#include "ScrollView.h"

#include <QApplication>
#include <QDebug>
#include <QPainter>
#include <QStyle>
#include <QStyleOptionSlider>
#include <QMenu>

namespace WebCore {

ScrollbarTheme* ScrollbarTheme::nativeTheme()
{
    static ScrollbarThemeQt theme;
    return &theme;
}

ScrollbarThemeQt::~ScrollbarThemeQt()
{
}

static QStyle::SubControl scPart(const ScrollbarPart& part)
{
    switch (part) {
        case NoPart:
            return QStyle::SC_None;
        case BackButtonStartPart:
        case BackButtonEndPart:
            return QStyle::SC_ScrollBarSubLine;
        case BackTrackPart:
            return QStyle::SC_ScrollBarSubPage;
        case ThumbPart:
            return QStyle::SC_ScrollBarSlider;
        case ForwardTrackPart:
            return QStyle::SC_ScrollBarAddPage;
        case ForwardButtonStartPart:
        case ForwardButtonEndPart:
            return QStyle::SC_ScrollBarAddLine;
    }

    return QStyle::SC_None;
}

static ScrollbarPart scrollbarPart(const QStyle::SubControl& sc)
{
    switch (sc) {
        case QStyle::SC_None:
            return NoPart;
        case QStyle::SC_ScrollBarSubLine:
            return BackButtonStartPart;
        case QStyle::SC_ScrollBarSubPage:
            return BackTrackPart;
        case QStyle::SC_ScrollBarSlider:
            return ThumbPart;
        case QStyle::SC_ScrollBarAddPage:
            return ForwardTrackPart;
        case QStyle::SC_ScrollBarAddLine:
            return ForwardButtonStartPart;
    }
    return NoPart;
}

static QStyleOptionSlider* styleOptionSlider(Scrollbar* scrollbar, QWidget* widget = 0)
{
    static QStyleOptionSlider opt;
    if (widget)
        opt.initFrom(widget);
    else
        opt.state |= QStyle::State_Active;

    opt.state &= ~QStyle::State_HasFocus;

    opt.rect = scrollbar->frameRect();
    if (scrollbar->enabled())
        opt.state |= QStyle::State_Enabled;
    if (scrollbar->controlSize() != RegularScrollbar)
        opt.state |= QStyle::State_Mini;
    opt.orientation = (scrollbar->orientation() == VerticalScrollbar) ? Qt::Vertical : Qt::Horizontal;
    if (scrollbar->orientation() == HorizontalScrollbar)
        opt.state |= QStyle::State_Horizontal;
    opt.sliderValue = scrollbar->value();
    opt.sliderPosition = opt.sliderValue;
    opt.pageStep = scrollbar->visibleSize();
    opt.singleStep = scrollbar->lineStep();
    opt.minimum = 0;
    opt.maximum = qMax(0, scrollbar->maximum());
    ScrollbarPart pressedPart = scrollbar->pressedPart();
    ScrollbarPart hoveredPart = scrollbar->hoveredPart();
    if (pressedPart != NoPart) {
        opt.activeSubControls = scPart(scrollbar->pressedPart());
        if (pressedPart == BackButtonStartPart || pressedPart == ForwardButtonStartPart ||
            pressedPart == BackButtonEndPart || pressedPart == ForwardButtonEndPart ||
            pressedPart == ThumbPart)
            opt.state |= QStyle::State_Sunken;
    } else
        opt.activeSubControls = scPart(hoveredPart);
    if (hoveredPart != NoPart)
        opt.state |= QStyle::State_MouseOver;
    return &opt;
}

bool ScrollbarThemeQt::paint(Scrollbar* scrollbar, GraphicsContext* graphicsContext, const IntRect& damageRect)
{
    if (graphicsContext->updatingControlTints()) {
       scrollbar->invalidateRect(damageRect);
       return false;
    }

    StylePainter p(this, graphicsContext);
    if (!p.isValid())
      return true;

    p.painter->save();
    QStyleOptionSlider* opt = styleOptionSlider(scrollbar, p.widget);

    p.painter->setClipRect(opt->rect.intersected(damageRect), Qt::IntersectClip);

#ifdef Q_WS_MAC
    p.drawComplexControl(QStyle::CC_ScrollBar, *opt);
#else
    const QPoint topLeft = opt->rect.topLeft();
    p.painter->translate(topLeft);
    opt->rect.moveTo(QPoint(0, 0));

    // The QStyle expects the background to be already filled
    p.painter->fillRect(opt->rect, opt->palette.background());

    p.drawComplexControl(QStyle::CC_ScrollBar, *opt);
    opt->rect.moveTo(topLeft);
#endif
    p.painter->restore();

    return true;
}

ScrollbarPart ScrollbarThemeQt::hitTest(Scrollbar* scrollbar, const PlatformMouseEvent& evt)
{
    QStyleOptionSlider* opt = styleOptionSlider(scrollbar);
    const QPoint pos = scrollbar->convertFromContainingWindow(evt.pos());
    opt->rect.moveTo(QPoint(0, 0));
    QStyle::SubControl sc = style()->hitTestComplexControl(QStyle::CC_ScrollBar, opt, pos, 0);
    return scrollbarPart(sc);
}

bool ScrollbarThemeQt::shouldCenterOnThumb(Scrollbar*, const PlatformMouseEvent& evt)
{
    // Middle click centers slider thumb (if supported)
    return style()->styleHint(QStyle::SH_ScrollBar_MiddleClickAbsolutePosition) && evt.button() == MiddleButton;
}

void ScrollbarThemeQt::invalidatePart(Scrollbar* scrollbar, ScrollbarPart)
{
    // FIXME: Do more precise invalidation.
    scrollbar->invalidate();
}

int ScrollbarThemeQt::scrollbarThickness(ScrollbarControlSize controlSize)
{
    QStyleOptionSlider o;
    o.orientation = Qt::Vertical;
    o.state &= ~QStyle::State_Horizontal;
    if (controlSize != RegularScrollbar)
        o.state |= QStyle::State_Mini;
    return style()->pixelMetric(QStyle::PM_ScrollBarExtent, &o, 0);
}

int ScrollbarThemeQt::thumbPosition(Scrollbar* scrollbar)
{
    if (scrollbar->enabled())
        return (int)((float)scrollbar->currentPos() * (trackLength(scrollbar) - thumbLength(scrollbar)) / scrollbar->maximum());
    return 0;
}

int ScrollbarThemeQt::thumbLength(Scrollbar* scrollbar)
{
    QStyleOptionSlider* opt = styleOptionSlider(scrollbar);
    IntRect thumb = style()->subControlRect(QStyle::CC_ScrollBar, opt, QStyle::SC_ScrollBarSlider, 0);
    return scrollbar->orientation() == HorizontalScrollbar ? thumb.width() : thumb.height();
}

int ScrollbarThemeQt::trackPosition(Scrollbar* scrollbar)
{
    QStyleOptionSlider* opt = styleOptionSlider(scrollbar);
    IntRect track = style()->subControlRect(QStyle::CC_ScrollBar, opt, QStyle::SC_ScrollBarGroove, 0);
    return scrollbar->orientation() == HorizontalScrollbar ? track.x() - scrollbar->x() : track.y() - scrollbar->y();
}

int ScrollbarThemeQt::trackLength(Scrollbar* scrollbar)
{
    QStyleOptionSlider* opt = styleOptionSlider(scrollbar);
    IntRect track = style()->subControlRect(QStyle::CC_ScrollBar, opt, QStyle::SC_ScrollBarGroove, 0);
    return scrollbar->orientation() == HorizontalScrollbar ? track.width() : track.height();
}

void ScrollbarThemeQt::paintScrollCorner(ScrollView* scrollView, GraphicsContext* context, const IntRect& rect)
{
    if (context->updatingControlTints()) {
       scrollView->invalidateRect(rect);
       return;
    }

#if QT_VERSION < 0x040500
    context->fillRect(rect, QApplication::palette().color(QPalette::Normal, QPalette::Window), DeviceColorSpace);
#else
    StylePainter p(this, context);
    if (!p.isValid())
        return;

    QStyleOption option;
    option.rect = rect;
    p.drawPrimitive(QStyle::PE_PanelScrollAreaCorner, option);
#endif
}

QStyle* ScrollbarThemeQt::style() const
{
    return QApplication::style();
}

}

