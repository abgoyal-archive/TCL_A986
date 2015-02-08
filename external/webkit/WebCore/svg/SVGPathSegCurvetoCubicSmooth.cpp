

#include "config.h"
#if ENABLE(SVG)
#include "SVGPathSegCurvetoCubicSmooth.h"

namespace WebCore {

SVGPathSegCurvetoCubicSmoothAbs::SVGPathSegCurvetoCubicSmoothAbs(float x, float y, float x2, float y2)
    : SVGPathSegCurvetoCubicSmooth(x, y, x2, y2)
{
}

SVGPathSegCurvetoCubicSmoothRel::SVGPathSegCurvetoCubicSmoothRel(float x, float y, float x2, float y2)
    : SVGPathSegCurvetoCubicSmooth(x, y, x2, y2)
{
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
