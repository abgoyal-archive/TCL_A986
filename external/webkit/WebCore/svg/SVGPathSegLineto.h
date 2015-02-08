

#ifndef SVGPathSegLineto_h
#define SVGPathSegLineto_h

#if ENABLE(SVG)

#include "SVGPathSeg.h"

namespace WebCore {
    class SVGPathSegLinetoAbs : public SVGPathSegSingleCoord { 
    public:
        static PassRefPtr<SVGPathSegLinetoAbs> create(float x, float y) { return adoptRef(new SVGPathSegLinetoAbs(x, y)); }

        virtual unsigned short pathSegType() const { return PATHSEG_LINETO_ABS; }
        virtual String pathSegTypeAsLetter() const { return "L"; }

    private:
        SVGPathSegLinetoAbs(float x, float y);
    };

    class SVGPathSegLinetoRel : public SVGPathSegSingleCoord { 
    public:
        static PassRefPtr<SVGPathSegLinetoRel> create(float x, float y) { return adoptRef(new SVGPathSegLinetoRel(x, y)); }


        virtual unsigned short pathSegType() const { return PATHSEG_LINETO_REL; }
        virtual String pathSegTypeAsLetter() const { return "l"; }

    private:
        SVGPathSegLinetoRel(float x, float y);
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif

// vim:ts=4:noet
