

#ifndef SVGPathSegLinetoHorizontal_h
#define SVGPathSegLinetoHorizontal_h

#if ENABLE(SVG)

#include "SVGPathSeg.h"

namespace WebCore {

    class SVGPathSegLinetoHorizontal : public SVGPathSeg {
    public:
        SVGPathSegLinetoHorizontal(float x) : SVGPathSeg(), m_x(x) {}

        virtual String toString() const { return pathSegTypeAsLetter() + String::format(" %.6lg", m_x); }

        void setX(float x) { m_x = x; }
        float x() const { return m_x; }

    private:
        float m_x;
    };

    class SVGPathSegLinetoHorizontalAbs : public SVGPathSegLinetoHorizontal {
    public:
        static PassRefPtr<SVGPathSegLinetoHorizontalAbs> create(float x) { return adoptRef(new SVGPathSegLinetoHorizontalAbs(x)); }

        virtual unsigned short pathSegType() const { return PATHSEG_LINETO_HORIZONTAL_ABS; }
        virtual String pathSegTypeAsLetter() const { return "H"; }

    private:
        SVGPathSegLinetoHorizontalAbs(float x);
    };

    class SVGPathSegLinetoHorizontalRel : public SVGPathSegLinetoHorizontal {
    public:
        static PassRefPtr<SVGPathSegLinetoHorizontalRel> create(float x) { return adoptRef(new SVGPathSegLinetoHorizontalRel(x)); }

        virtual unsigned short pathSegType() const { return PATHSEG_LINETO_HORIZONTAL_REL; }
        virtual String pathSegTypeAsLetter() const { return "h"; }

    private:
        SVGPathSegLinetoHorizontalRel(float x);
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif

// vim:ts=4:noet
