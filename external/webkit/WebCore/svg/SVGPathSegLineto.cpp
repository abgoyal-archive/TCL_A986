

#include "config.h"

#if ENABLE(SVG)
#include "SVGPathSegLineto.h"

namespace WebCore {

SVGPathSegLinetoAbs::SVGPathSegLinetoAbs(float x, float y)
    : SVGPathSegSingleCoord(x, y)
{
}

SVGPathSegLinetoRel::SVGPathSegLinetoRel(float x, float y)
    : SVGPathSegSingleCoord(x, y)
{
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
