

#include "config.h"

#if ENABLE(SVG)
#include "SVGPathSegCurvetoQuadraticSmooth.h"

namespace WebCore {

SVGPathSegCurvetoQuadraticSmoothAbs::SVGPathSegCurvetoQuadraticSmoothAbs(float x, float y)
    : SVGPathSegSingleCoord(x, y)
{
}

SVGPathSegCurvetoQuadraticSmoothRel::SVGPathSegCurvetoQuadraticSmoothRel(float x, float y)
    : SVGPathSegSingleCoord(x, y)
{
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
