

// All of the functions in this file should move to new homes and this file should be deleted.

#ifndef SkiaUtils_h
#define SkiaUtils_h

#include <wtf/MathExtras.h>
#include "GraphicsContext.h"
#include "SkPath.h"
#include "SkXfermode.h"

class SkCanvas;
class SkRegion;

namespace WebCore {

SkXfermode::Mode WebCoreCompositeToSkiaComposite(CompositeOperator);

// move this guy into SkColor.h
SkColor SkPMColorToColor(SkPMColor);

// This should be an operator on Color
Color SkPMColorToWebCoreColor(SkPMColor);

// Skia has problems when passed infinite, etc floats, filter them to 0.
inline SkScalar WebCoreFloatToSkScalar(float f)
{
    return SkFloatToScalar(isfinite(f) ? f : 0);
}

inline SkScalar WebCoreDoubleToSkScalar(double d)
{
    return SkDoubleToScalar(isfinite(d) ? d : 0);
}

// Computes the smallest rectangle that, which when drawn to the given canvas,
// will cover the same area as the source rectangle. It will clip to the canvas'
// clip, doing the necessary coordinate transforms.
//
// srcRect and destRect can be the same.
void ClipRectToCanvas(const SkCanvas&, const SkRect& srcRect, SkRect* destRect);

// Determine if a given WebKit point is contained in a path
bool SkPathContainsPoint(SkPath*, const FloatPoint&, SkPath::FillType);

// Returns a statically allocated 1x1 GraphicsContext intended for temporary
// operations. Please save() the state and restore() it when you're done with
// the context.
GraphicsContext* scratchContext();

}  // namespace WebCore

#endif  // SkiaUtils_h
