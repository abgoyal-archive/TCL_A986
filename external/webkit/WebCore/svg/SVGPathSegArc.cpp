

#include "config.h"

#if ENABLE(SVG)
#include "SVGPathSegArc.h"

namespace WebCore {

SVGPathSegArcAbs::SVGPathSegArcAbs(float x, float y, float r1, float r2, float angle, bool largeArcFlag, bool sweepFlag)
    : SVGPathSegArc(x, y, r1, r2, angle, largeArcFlag, sweepFlag)
{
}

SVGPathSegArcRel::SVGPathSegArcRel(float x, float y, float r1, float r2, float angle, bool largeArcFlag, bool sweepFlag)
    : SVGPathSegArc(x, y, r1, r2, angle, largeArcFlag, sweepFlag)
{
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
