

#ifndef SVGPathSegMoveto_h
#define SVGPathSegMoveto_h

#if ENABLE(SVG)

#include "SVGPathSeg.h"

namespace WebCore {
    class SVGPathSegMovetoAbs : public SVGPathSegSingleCoord { 
    public:
        static PassRefPtr<SVGPathSegMovetoAbs> create(float x, float y) { return adoptRef(new SVGPathSegMovetoAbs(x, y)); }

        virtual unsigned short pathSegType() const { return PATHSEG_MOVETO_ABS; }
        virtual String pathSegTypeAsLetter() const { return "M"; }

    private:
        SVGPathSegMovetoAbs(float x, float y);
    };

    class SVGPathSegMovetoRel : public SVGPathSegSingleCoord { 
    public:
        static PassRefPtr<SVGPathSegMovetoRel> create(float x, float y) { return adoptRef(new SVGPathSegMovetoRel(x, y)); }

        virtual unsigned short pathSegType() const { return PATHSEG_MOVETO_REL; }
        virtual String pathSegTypeAsLetter() const { return "m"; }

    private:
        SVGPathSegMovetoRel(float x, float y);
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif

// vim:ts=4:noet
