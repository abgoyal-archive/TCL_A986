

#include "config.h"

#if ENABLE(SVG)
#include "SVGPathSegCurvetoCubic.h"

namespace WebCore {

SVGPathSegCurvetoCubicAbs::SVGPathSegCurvetoCubicAbs(float x, float y, float x1, float y1, float x2, float y2)
    : SVGPathSegCurvetoCubic(x, y, x1, y1, x2, y2)
{
}

SVGPathSegCurvetoCubicRel::SVGPathSegCurvetoCubicRel(float x, float y, float x1, float y1, float x2, float y2)
    : SVGPathSegCurvetoCubic(x, y, x1, y1, x2, y2)
{
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
