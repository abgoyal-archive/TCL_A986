

#ifndef SVGPathSegLinetoVertical_h
#define SVGPathSegLinetoVertical_h

#if ENABLE(SVG)

#include "SVGPathSeg.h"

namespace WebCore {

    class SVGPathSegLinetoVertical : public SVGPathSeg {
    public:
        SVGPathSegLinetoVertical(float y) : SVGPathSeg(), m_y(y) {}

        virtual String toString() const { return pathSegTypeAsLetter() + String::format(" %.6lg", m_y); }

        void setY(float y) { m_y = y; }
        float y() const { return m_y; }

    private:
        float m_y;
    };

    class SVGPathSegLinetoVerticalAbs : public SVGPathSegLinetoVertical {
    public:
        static PassRefPtr<SVGPathSegLinetoVerticalAbs> create(float y) { return adoptRef(new SVGPathSegLinetoVerticalAbs(y)); }

        virtual unsigned short pathSegType() const { return PATHSEG_LINETO_VERTICAL_ABS; }
        virtual String pathSegTypeAsLetter() const { return "V"; }

    private:
        SVGPathSegLinetoVerticalAbs(float y);
    };

    class SVGPathSegLinetoVerticalRel : public SVGPathSegLinetoVertical {
    public:
        static PassRefPtr<SVGPathSegLinetoVerticalRel> create(float y) { return adoptRef(new SVGPathSegLinetoVerticalRel(y)); }

        virtual unsigned short pathSegType() const { return PATHSEG_LINETO_VERTICAL_REL; }
        virtual String pathSegTypeAsLetter() const { return "v"; }

    private:
        SVGPathSegLinetoVerticalRel(float y);
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif

// vim:ts=4:noet
