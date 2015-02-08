

#include "config.h"

#if ENABLE(SVG)
#include "SVGPathSegCurvetoQuadratic.h"

namespace WebCore {

SVGPathSegCurvetoQuadraticAbs::SVGPathSegCurvetoQuadraticAbs(float x, float y, float x1, float y1)
    : SVGPathSegCurvetoQuadratic(x, y, x1, y1)
{
}

SVGPathSegCurvetoQuadraticRel::SVGPathSegCurvetoQuadraticRel(float x, float y, float x1, float y1)
    : SVGPathSegCurvetoQuadratic(x, y, x1, y1)
{
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
