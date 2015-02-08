

#include "config.h"
#if ENABLE(SVG)
#include "SVGRenderStyleDefs.h"

#include "RenderStyle.h"
#include "SVGRenderStyle.h"

using namespace WebCore;

StyleFillData::StyleFillData()
{
    paint = SVGRenderStyle::initialFillPaint();
    opacity = SVGRenderStyle::initialFillOpacity();
}

StyleFillData::StyleFillData(const StyleFillData& other)
    : RefCounted<StyleFillData>()
{
    paint = other.paint;
    opacity = other.opacity;
}

bool StyleFillData::operator==(const StyleFillData &other) const
{
    if (opacity != other.opacity)
        return false;

    if (!paint || !other.paint)
        return paint == other.paint;

    if (paint->paintType() != other.paint->paintType())
        return false;

    if (paint->paintType() == SVGPaint::SVG_PAINTTYPE_URI)
        return paint->uri() == other.paint->uri();

    if (paint->paintType() == SVGPaint::SVG_PAINTTYPE_RGBCOLOR)
        return paint->color() == other.paint->color();

    return paint == other.paint;
}

StyleStrokeData::StyleStrokeData()
{
    width = SVGRenderStyle::initialStrokeWidth();
    paint = SVGRenderStyle::initialStrokePaint();
    opacity = SVGRenderStyle::initialStrokeOpacity();
    miterLimit = SVGRenderStyle::initialStrokeMiterLimit();
    dashOffset = SVGRenderStyle::initialStrokeDashOffset();
    dashArray = SVGRenderStyle::initialStrokeDashArray();
}

StyleStrokeData::StyleStrokeData(const StyleStrokeData& other)
    : RefCounted<StyleStrokeData>()
{
    width = other.width;
    paint = other.paint;
    opacity = other.opacity;
    miterLimit = other.miterLimit;
    dashOffset = other.dashOffset;
    dashArray = other.dashArray;
}

bool StyleStrokeData::operator==(const StyleStrokeData &other) const
{
    return (paint == other.paint) &&
           (width == other.width) &&
           (opacity == other.opacity) &&
           (miterLimit == other.miterLimit) &&
           (dashOffset == other.dashOffset) &&
           (dashArray == other.dashArray);
}

StyleStopData::StyleStopData()
{
    color = SVGRenderStyle::initialStopColor();
    opacity = SVGRenderStyle::initialStopOpacity();
}

StyleStopData::StyleStopData(const StyleStopData& other)
    : RefCounted<StyleStopData>()
{
    color = other.color;
    opacity = other.opacity;
}

bool StyleStopData::operator==(const StyleStopData &other) const
{
    return (color == other.color) &&
           (opacity == other.opacity);
}

StyleTextData::StyleTextData()
{
    kerning = SVGRenderStyle::initialKerning();
}

StyleTextData::StyleTextData(const StyleTextData& other)
    : RefCounted<StyleTextData>()
{
    kerning = other.kerning;
}

bool StyleTextData::operator==(const StyleTextData& other) const
{
    return kerning == other.kerning;
}

StyleClipData::StyleClipData()
{
    clipPath = SVGRenderStyle::initialClipPath();
}

StyleClipData::StyleClipData(const StyleClipData& other)
    : RefCounted<StyleClipData>()
{
    clipPath = other.clipPath;
}

bool StyleClipData::operator==(const StyleClipData &other) const
{
    return (clipPath == other.clipPath);
}

StyleMaskData::StyleMaskData()
{
    maskElement = SVGRenderStyle::initialMaskElement();
}

StyleMaskData::StyleMaskData(const StyleMaskData& other)
    : RefCounted<StyleMaskData>()
{
    maskElement = other.maskElement;
}

bool StyleMaskData::operator==(const StyleMaskData &other) const
{
    return (maskElement == other.maskElement);
}

StyleMarkerData::StyleMarkerData()
{
    startMarker = SVGRenderStyle::initialStartMarker();
    midMarker = SVGRenderStyle::initialMidMarker();
    endMarker = SVGRenderStyle::initialEndMarker();
}

StyleMarkerData::StyleMarkerData(const StyleMarkerData& other)
    : RefCounted<StyleMarkerData>()
{
    startMarker = other.startMarker;
    midMarker = other.midMarker;
    endMarker = other.endMarker;
}

bool StyleMarkerData::operator==(const StyleMarkerData &other) const
{
    return (startMarker == other.startMarker && midMarker == other.midMarker && endMarker == other.endMarker);
}

StyleMiscData::StyleMiscData()
{
    floodColor = SVGRenderStyle::initialFloodColor();
    floodOpacity = SVGRenderStyle::initialFloodOpacity();
    lightingColor = SVGRenderStyle::initialLightingColor();
    baselineShiftValue = SVGRenderStyle::initialBaselineShiftValue();
}

StyleMiscData::StyleMiscData(const StyleMiscData& other)
    : RefCounted<StyleMiscData>()
{
    filter = other.filter;
    floodColor = other.floodColor;
    floodOpacity = other.floodOpacity;
    lightingColor = other.lightingColor;
    baselineShiftValue = other.baselineShiftValue;
}

bool StyleMiscData::operator==(const StyleMiscData &other) const
{
    return filter == other.filter
           && floodOpacity == other.floodOpacity
           && floodColor == other.floodColor
           && lightingColor == other.lightingColor
           && baselineShiftValue == other.baselineShiftValue;
}

StyleShadowSVGData::StyleShadowSVGData()
{
}

StyleShadowSVGData::StyleShadowSVGData(const StyleShadowSVGData& other)
    : RefCounted<StyleShadowSVGData>()
    , shadow(other.shadow ? new ShadowData(*other.shadow) : 0)
{
}

bool StyleShadowSVGData::operator==(const StyleShadowSVGData& other) const
{
    if ((!shadow && other.shadow) || (shadow && !other.shadow))
        return false;
    if (shadow && other.shadow && (*shadow != *other.shadow))
        return false;
    return true;
}

#endif // ENABLE(SVG)

// vim:ts=4:noet
