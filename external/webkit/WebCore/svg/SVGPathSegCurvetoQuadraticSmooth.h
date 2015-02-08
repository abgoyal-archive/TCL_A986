

#ifndef SVGPathSegCurvetoQuadraticSmooth_h
#define SVGPathSegCurvetoQuadraticSmooth_h

#if ENABLE(SVG)

#include "SVGPathSeg.h"

namespace WebCore {

    class SVGPathSegCurvetoQuadraticSmoothAbs : public SVGPathSegSingleCoord {
    public:
        static PassRefPtr<SVGPathSegCurvetoQuadraticSmoothAbs> create(float x, float y) { return adoptRef(new SVGPathSegCurvetoQuadraticSmoothAbs(x, y)); }

        virtual unsigned short pathSegType() const { return PATHSEG_CURVETO_QUADRATIC_SMOOTH_ABS; }
        virtual String pathSegTypeAsLetter() const { return "T"; }

    private:
        SVGPathSegCurvetoQuadraticSmoothAbs(float x, float y);
    };

    class SVGPathSegCurvetoQuadraticSmoothRel : public SVGPathSegSingleCoord {
    public:
        static PassRefPtr<SVGPathSegCurvetoQuadraticSmoothRel> create(float x, float y) { return adoptRef(new SVGPathSegCurvetoQuadraticSmoothRel(x, y)); }

        virtual unsigned short pathSegType() const { return PATHSEG_CURVETO_QUADRATIC_SMOOTH_REL; }
        virtual String pathSegTypeAsLetter() const { return "t"; }

    private:
        SVGPathSegCurvetoQuadraticSmoothRel(float x, float y);
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif

// vim:ts=4:noet
