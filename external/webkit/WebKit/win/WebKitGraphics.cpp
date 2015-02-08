

#include "config.h"
#include "WebKitGraphics.h"

#include "WebKit.h"
#include "WebKitDLL.h"

#include "WebPreferences.h"

#pragma warning(push, 0)
#include <WebCore/CharacterNames.h>
#include <WebCore/Font.h>
#include <WebCore/FontDatabase.h>
#include <WebCore/FontDescription.h>
#include <WebCore/FontSelector.h>
#include <WebCore/GraphicsContext.h>
#include <WebCore/PlatformString.h>
#include <WebCore/StringTruncator.h>
#include <WebCore/WebCoreTextRenderer.h>

#include <CoreGraphics/CoreGraphics.h>
#pragma warning(pop)

#include <WebKitSystemInterface/WebKitSystemInterface.h>

using namespace WebCore;

static Font makeFont(const WebFontDescription& description)
{
    AtomicString::init();
    populateFontDatabase();

    String fontFamilyString(description.family, description.familyLength);

    FontDescription f;
    FontFamily family;
    family.setFamily(fontFamilyString);
    f.setFamily(family);
    f.setSpecifiedSize(description.size);
    f.setComputedSize(description.size);
    f.setItalic(description.italic);
    f.setWeight(description.bold ? FontWeightBold : FontWeightNormal);
    f.setIsAbsoluteSize(true);

    FontSmoothingType smoothingType;
    if (SUCCEEDED(WebPreferences::sharedStandardPreferences()->fontSmoothing(&smoothingType)))
        f.setRenderingMode(smoothingType == FontSmoothingTypeWindows ? AlternateRenderingMode : NormalRenderingMode);

    Font font(f, 0, 0);
    font.update(0);

    return font;
}

// Text shadow is added post 3.1.1.  In order for nightlies to not break Safari 3.1.1, we should still allow
// the old WebTextRenderInfo that has a smaller structSize than the current one with the new text shadow data members.
struct WebTextRenderInfoWithoutShadow
{
    DWORD structSize;
    CGContextRef cgContext;
    LPCTSTR text;
    int length;
    POINT pt;
    const WebFontDescription* description;
    CGColorRef color;
    int underlinedIndex;
    bool drawAsPassword;
    int overrideSmoothingLevel; // pass in -1 if caller does not want to override smoothing level
};

void WebDrawText(WebTextRenderInfo* info)
{
    if (!info || info->structSize < sizeof(WebTextRenderInfoWithoutShadow) || !info->cgContext || !info->description)
        return;

    int oldFontSmoothingLevel = -1;
    if (info->overrideSmoothingLevel >= 0) {
        oldFontSmoothingLevel = wkGetFontSmoothingLevel();
        wkSetFontSmoothingLevel(info->overrideSmoothingLevel);
    }

    {
        GraphicsContext context(info->cgContext);
        String drawString(info->text, info->length);
        if (info->drawAsPassword)
            drawString = drawString.impl()->secure(WebCore::bullet);

        context.save();

        // Set shadow setting
        if (info->structSize == sizeof(WebTextRenderInfo) &&
            (info->shadowOffset.cx || info->shadowOffset.cy || info->shadowBlur || info->shadowColor))
            context.setShadow(info->shadowOffset, info->shadowBlur, info->shadowColor, DeviceColorSpace);

        WebCoreDrawTextAtPoint(context, drawString, info->pt, makeFont(*(info->description)), info->color, info->underlinedIndex);
        context.restore();
    }

    if (info->overrideSmoothingLevel >= 0)
        wkSetFontSmoothingLevel(oldFontSmoothingLevel);
}

float TextFloatWidth(LPCTSTR text, int length, const WebFontDescription& description)
{
    return WebCoreTextFloatWidth(String(text, length), makeFont(description));
}

void FontMetrics(const WebFontDescription& description, int* ascent, int* descent, int* lineSpacing)
{
    if (!ascent && !descent && !lineSpacing)
        return;

    Font font(makeFont(description));

    if (ascent)
        *ascent = font.ascent();

    if (descent)
        *descent = font.descent();

    if (lineSpacing)
        *lineSpacing = font.lineSpacing();
}

unsigned CenterTruncateStringToWidth(LPCTSTR text, int length, const WebFontDescription& description, float width, WCHAR* buffer)
{
    ASSERT(buffer);

    String result = StringTruncator::centerTruncate(String(text, length), width, makeFont(description), false);
    memcpy(buffer, result.characters(), result.length() * sizeof(UChar));
    buffer[result.length()] = '\0';
    return result.length();
}

unsigned RightTruncateStringToWidth(LPCTSTR text, int length, const WebFontDescription& description, float width, WCHAR* buffer)
{
    ASSERT(buffer);

    String result = StringTruncator::rightTruncate(String(text, length), width, makeFont(description), false);
    memcpy(buffer, result.characters(), result.length() * sizeof(UChar));
    buffer[result.length()] = '\0';
    return result.length();
}

void WebKitSetShouldUseFontSmoothing(bool smooth)
{
    WebCoreSetShouldUseFontSmoothing(smooth);
}

bool WebKitShouldUseFontSmoothing()
{
    return WebCoreShouldUseFontSmoothing();
}
