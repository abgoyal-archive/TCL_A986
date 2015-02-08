

#include "config.h"

#if ENABLE(SVG)
#include "SVGPathSegLinetoHorizontal.h"

namespace WebCore {

SVGPathSegLinetoHorizontalAbs::SVGPathSegLinetoHorizontalAbs(float x)
    : SVGPathSegLinetoHorizontal(x)
{
}

SVGPathSegLinetoHorizontalRel::SVGPathSegLinetoHorizontalRel(float x)
    : SVGPathSegLinetoHorizontal(x)
{
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
