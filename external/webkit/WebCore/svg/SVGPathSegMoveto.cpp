

#include "config.h"

#if ENABLE(SVG)
#include "SVGPathSegMoveto.h"

namespace WebCore {

SVGPathSegMovetoAbs::SVGPathSegMovetoAbs(float x, float y)
    : SVGPathSegSingleCoord(x, y)
{
}

SVGPathSegMovetoRel::SVGPathSegMovetoRel(float x, float y)
    : SVGPathSegSingleCoord(x, y)
{
}
}

#endif // ENABLE(SVG)

// vim:ts=4:noet
