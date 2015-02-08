

#include "config.h"
#include "RenderThemeHaiku.h"

#include "GraphicsContext.h"
#include "NotImplemented.h"
#include <ControlLook.h>
#include <View.h>


namespace WebCore {

PassRefPtr<RenderTheme> RenderThemeHaiku::create()
{
    return adoptRef(new RenderThemeHaiku());
}

PassRefPtr<RenderTheme> RenderTheme::themeForPage(Page*)
{
    static RenderTheme* renderTheme = RenderThemeHaiku::create().releaseRef();
    return renderTheme;
}

RenderThemeHaiku::RenderThemeHaiku()
{
}

RenderThemeHaiku::~RenderThemeHaiku()
{
}

static bool supportsFocus(ControlPart appearance)
{
    switch (appearance) {
    case PushButtonPart:
    case ButtonPart:
    case TextFieldPart:
    case TextAreaPart:
    case SearchFieldPart:
    case MenulistPart:
    case RadioPart:
    case CheckboxPart:
        return true;
    default:
        return false;
    }
}

bool RenderThemeHaiku::supportsFocusRing(const RenderStyle* style) const
{
    return supportsFocus(style->appearance());
}

Color RenderThemeHaiku::platformActiveSelectionBackgroundColor() const
{
    return Color(ui_color(B_CONTROL_HIGHLIGHT_COLOR));
}

Color RenderThemeHaiku::platformInactiveSelectionBackgroundColor() const
{
    return Color(ui_color(B_CONTROL_HIGHLIGHT_COLOR));
}

Color RenderThemeHaiku::platformActiveSelectionForegroundColor() const
{
    return Color(ui_color(B_CONTROL_TEXT_COLOR));
}

Color RenderThemeHaiku::platformInactiveSelectionForegroundColor() const
{
    return Color(ui_color(B_CONTROL_TEXT_COLOR));
}

Color RenderThemeHaiku::platformTextSearchHighlightColor() const
{
    return Color(ui_color(B_MENU_SELECTED_BACKGROUND_COLOR));
}

void RenderThemeHaiku::systemFont(int propId, FontDescription&) const
{
    notImplemented();
}

bool RenderThemeHaiku::paintCheckbox(RenderObject*, const RenderObject::PaintInfo& info, const IntRect& intRect)
{
    if (info.context->paintingDisabled())
        return false;

    rgb_color base = ui_color(B_PANEL_BACKGROUND_COLOR);
    BRect rect = intRect;
    BView* view = info.context->platformContext();

    if (!be_control_look)
        return false;

    be_control_look->DrawCheckBox(view, rect, rect, base);
    return true;
}

void RenderThemeHaiku::setCheckboxSize(RenderStyle* style) const
{
    int size = 10;

    // If the width and height are both specified, then we have nothing to do.
    if (!style->width().isIntrinsicOrAuto() && !style->height().isAuto())
        return;

    // FIXME: A hard-coded size of 'size' is used. This is wrong but necessary for now.
    if (style->width().isIntrinsicOrAuto())
        style->setWidth(Length(size, Fixed));

    if (style->height().isAuto())
        style->setHeight(Length(size, Fixed));
}

bool RenderThemeHaiku::paintRadio(RenderObject*, const RenderObject::PaintInfo& info, const IntRect& intRect)
{
    if (info.context->paintingDisabled())
        return false;

    rgb_color base = ui_color(B_PANEL_BACKGROUND_COLOR);
    BRect rect = intRect;
    BView* view = info.context->platformContext();

    if (!be_control_look)
        return false;

    be_control_look->DrawRadioButton(view, rect, rect, base);
    return true;
}

void RenderThemeHaiku::setRadioSize(RenderStyle* style) const
{
    // This is the same as checkboxes.
    setCheckboxSize(style);
}

void RenderThemeHaiku::adjustMenuListStyle(CSSStyleSelector*, RenderStyle* style, Element*) const
{
    // Leave some space for the arrow.
    style->setPaddingRight(Length(22, Fixed));
    const int minHeight = 20;
    style->setMinHeight(Length(minHeight, Fixed));
}

bool RenderThemeHaiku::paintMenuList(RenderObject*, const RenderObject::PaintInfo&, const IntRect&)
{
    notImplemented();
    return false;
}

} // namespace WebCore
