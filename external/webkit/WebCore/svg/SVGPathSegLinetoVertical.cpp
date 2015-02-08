

#include "config.h"

#if ENABLE(SVG)
#include "SVGPathSegLinetoVertical.h"

namespace WebCore {

SVGPathSegLinetoVerticalAbs::SVGPathSegLinetoVerticalAbs(float y)
    : SVGPathSegLinetoVertical(y)
{
}

SVGPathSegLinetoVerticalRel::SVGPathSegLinetoVerticalRel(float y)
    : SVGPathSegLinetoVertical(y)
{
}
}

#endif // ENABLE(SVG)

// vim:ts=4:noet
