
#include "config.h"
#include "WebCoreTextRenderer.h"

#include "Font.h"
#include "FontDescription.h"
#include "GraphicsContext.h"
#include "StringTruncator.h"
#include <wtf/unicode/Unicode.h>

namespace WebCore {

static bool shouldUseFontSmoothing = true;

static bool isOneLeftToRightRun(const TextRun& run)
{
    for (int i = 0; i < run.length(); i++) {
        WTF::Unicode::Direction direction = WTF::Unicode::direction(run[i]);
        if (direction == WTF::Unicode::RightToLeft || direction > WTF::Unicode::OtherNeutral)
            return false;
    }
    return true;
}

static void doDrawTextAtPoint(GraphicsContext& context, const String& text, const IntPoint& point, const Font& font, const Color& color, int underlinedIndex)
{
    TextRun run(text.characters(), text.length());

    context.setFillColor(color, DeviceColorSpace);
    if (isOneLeftToRightRun(run))
        font.drawText(&context, run, point);
    else
        context.drawBidiText(font, run, point);

    if (underlinedIndex >= 0) {
        ASSERT(underlinedIndex < static_cast<int>(text.length()));

        int beforeWidth;
        if (underlinedIndex > 0) {
            TextRun beforeRun(text.characters(), underlinedIndex);
            beforeWidth = font.width(beforeRun);
        } else
            beforeWidth = 0;

        TextRun underlinedRun(text.characters() + underlinedIndex, 1);
        int underlinedWidth = font.width(underlinedRun);

        IntPoint underlinePoint(point);
        underlinePoint.move(beforeWidth, 1);

        context.setStrokeColor(color, DeviceColorSpace);
        context.drawLineForText(underlinePoint, underlinedWidth, false);
    }
}

void WebCoreDrawTextAtPoint(GraphicsContext& context, const String& text, const IntPoint& point, const Font& font, const Color& color, int underlinedIndex)
{
    context.save();

    doDrawTextAtPoint(context, text, point, font, color, underlinedIndex);

    context.restore();
}

void WebCoreDrawDoubledTextAtPoint(GraphicsContext& context, const String& text, const IntPoint& point, const Font& font, const Color& topColor, const Color& bottomColor, int underlinedIndex)
{
    context.save();

    IntPoint textPos = point;

    doDrawTextAtPoint(context, text, textPos, font, bottomColor, underlinedIndex);
    textPos.move(0, -1);
    doDrawTextAtPoint(context, text, textPos, font, topColor, underlinedIndex);

    context.restore();
}

float WebCoreTextFloatWidth(const String& text, const Font& font)
{
    return StringTruncator::width(text, font, false);
}

void WebCoreSetShouldUseFontSmoothing(bool smooth)
{
    shouldUseFontSmoothing = smooth;
}

bool WebCoreShouldUseFontSmoothing()
{
    return shouldUseFontSmoothing;
}

void WebCoreSetAlwaysUsesComplexTextCodePath(bool complex)
{
    Font::setCodePath(complex ? Font::Complex : Font::Auto);
}

bool WebCoreAlwaysUsesComplexTextCodePath()
{
    return Font::codePath() == Font::Complex;
}

} // namespace WebCore
