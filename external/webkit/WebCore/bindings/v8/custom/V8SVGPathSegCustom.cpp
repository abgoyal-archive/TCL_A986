

#include "config.h"
#include "V8SVGPathSeg.h"

#include "V8DOMWindow.h"
#include "V8DOMWrapper.h"
#include "V8SVGPathSegArcAbs.h"
#include "V8SVGPathSegArcRel.h"
#include "V8SVGPathSegClosePath.h"
#include "V8SVGPathSegCurvetoCubicAbs.h"
#include "V8SVGPathSegCurvetoCubicRel.h"
#include "V8SVGPathSegCurvetoCubicSmoothAbs.h"
#include "V8SVGPathSegCurvetoCubicSmoothRel.h"
#include "V8SVGPathSegCurvetoQuadraticAbs.h"
#include "V8SVGPathSegCurvetoQuadraticRel.h"
#include "V8SVGPathSegCurvetoQuadraticSmoothAbs.h"
#include "V8SVGPathSegCurvetoQuadraticSmoothRel.h"
#include "V8SVGPathSegLinetoAbs.h"
#include "V8SVGPathSegLinetoHorizontalAbs.h"
#include "V8SVGPathSegLinetoHorizontalRel.h"
#include "V8SVGPathSegLinetoRel.h"
#include "V8SVGPathSegLinetoVerticalAbs.h"
#include "V8SVGPathSegLinetoVerticalRel.h"
#include "V8SVGPathSegMovetoAbs.h"
#include "V8SVGPathSegMovetoRel.h"

namespace WebCore {

v8::Handle<v8::Value> toV8(SVGPathSeg* impl)
{
    if (!impl)
        return v8::Null();
    switch (impl->pathSegType()) {
    case SVGPathSeg::PATHSEG_CLOSEPATH:
        return toV8(static_cast<SVGPathSegClosePath*>(impl));
    case SVGPathSeg::PATHSEG_MOVETO_ABS:
        return toV8(static_cast<SVGPathSegMovetoAbs*>(impl));
    case SVGPathSeg::PATHSEG_MOVETO_REL:
        return toV8(static_cast<SVGPathSegMovetoRel*>(impl));
    case SVGPathSeg::PATHSEG_LINETO_ABS:
        return toV8(static_cast<SVGPathSegLinetoAbs*>(impl));
    case SVGPathSeg::PATHSEG_LINETO_REL:
        return toV8(static_cast<SVGPathSegLinetoRel*>(impl));
    case SVGPathSeg::PATHSEG_CURVETO_CUBIC_ABS:
        return toV8(static_cast<SVGPathSegCurvetoCubicAbs*>(impl));
    case SVGPathSeg::PATHSEG_CURVETO_CUBIC_REL:
        return toV8(static_cast<SVGPathSegCurvetoCubicRel*>(impl));
    case SVGPathSeg::PATHSEG_CURVETO_QUADRATIC_ABS:
        return toV8(static_cast<SVGPathSegCurvetoQuadraticAbs*>(impl));
    case SVGPathSeg::PATHSEG_CURVETO_QUADRATIC_REL:
        return toV8(static_cast<SVGPathSegCurvetoQuadraticRel*>(impl));
    case SVGPathSeg::PATHSEG_ARC_ABS:
        return toV8(static_cast<SVGPathSegArcAbs*>(impl));
    case SVGPathSeg::PATHSEG_ARC_REL:
        return toV8(static_cast<SVGPathSegArcRel*>(impl));
    case SVGPathSeg::PATHSEG_LINETO_HORIZONTAL_ABS:
        return toV8(static_cast<SVGPathSegLinetoHorizontalAbs*>(impl));
    case SVGPathSeg::PATHSEG_LINETO_HORIZONTAL_REL:
        return toV8(static_cast<SVGPathSegLinetoHorizontalRel*>(impl));
    case SVGPathSeg::PATHSEG_LINETO_VERTICAL_ABS:
        return toV8(static_cast<SVGPathSegLinetoVerticalAbs*>(impl));
    case SVGPathSeg::PATHSEG_LINETO_VERTICAL_REL:
        return toV8(static_cast<SVGPathSegLinetoVerticalRel*>(impl));
    case SVGPathSeg::PATHSEG_CURVETO_CUBIC_SMOOTH_ABS:
        return toV8(static_cast<SVGPathSegCurvetoCubicSmoothAbs*>(impl));
    case SVGPathSeg::PATHSEG_CURVETO_CUBIC_SMOOTH_REL:
        return toV8(static_cast<SVGPathSegCurvetoCubicSmoothRel*>(impl));
    case SVGPathSeg::PATHSEG_CURVETO_QUADRATIC_SMOOTH_ABS:
        return toV8(static_cast<SVGPathSegCurvetoQuadraticSmoothAbs*>(impl));
    case SVGPathSeg::PATHSEG_CURVETO_QUADRATIC_SMOOTH_REL:
        return toV8(static_cast<SVGPathSegCurvetoQuadraticSmoothRel*>(impl));
    }
    ASSERT_NOT_REACHED();
    return V8SVGPathSeg::wrap(impl);
}

} // namespace WebCore
