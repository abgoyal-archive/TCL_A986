

#ifndef SVGPathSeg_h
#define SVGPathSeg_h

#if ENABLE(SVG)
#include "PlatformString.h"
#include "SVGNames.h"

#include <wtf/RefCounted.h>

namespace WebCore {
    class SVGPathElement;
    class SVGStyledElement;

    class SVGPathSeg : public RefCounted<SVGPathSeg> {
    public:
        virtual ~SVGPathSeg() { }

        enum SVGPathSegType {
            PATHSEG_UNKNOWN                         = 0,
            PATHSEG_CLOSEPATH                       = 1,
            PATHSEG_MOVETO_ABS                      = 2,
            PATHSEG_MOVETO_REL                      = 3,
            PATHSEG_LINETO_ABS                      = 4,
            PATHSEG_LINETO_REL                      = 5,
            PATHSEG_CURVETO_CUBIC_ABS               = 6,
            PATHSEG_CURVETO_CUBIC_REL               = 7,
            PATHSEG_CURVETO_QUADRATIC_ABS           = 8,
            PATHSEG_CURVETO_QUADRATIC_REL           = 9,
            PATHSEG_ARC_ABS                         = 10,
            PATHSEG_ARC_REL                         = 11,
            PATHSEG_LINETO_HORIZONTAL_ABS           = 12,
            PATHSEG_LINETO_HORIZONTAL_REL           = 13,
            PATHSEG_LINETO_VERTICAL_ABS             = 14,
            PATHSEG_LINETO_VERTICAL_REL             = 15,
            PATHSEG_CURVETO_CUBIC_SMOOTH_ABS        = 16,
            PATHSEG_CURVETO_CUBIC_SMOOTH_REL        = 17,
            PATHSEG_CURVETO_QUADRATIC_SMOOTH_ABS    = 18,
            PATHSEG_CURVETO_QUADRATIC_SMOOTH_REL    = 19
        };

        virtual unsigned short pathSegType() const { return PATHSEG_UNKNOWN; }
        virtual String pathSegTypeAsLetter() const { return ""; }
        virtual String toString() const { return ""; }

        const QualifiedName& associatedAttributeName() const { return SVGNames::dAttr; }
        
    protected:
        SVGPathSeg() { }
    };

    class SVGPathSegSingleCoord : public SVGPathSeg { 
    public:
        SVGPathSegSingleCoord(float x, float y)
        : SVGPathSeg() , m_x(x) , m_y(y) {}

        void setX(float x) { m_x = x; }
        float x() const { return m_x; }

        void setY(float y) { m_y = y; }
        float y() const { return m_y; }

        virtual String toString() const { return pathSegTypeAsLetter() + String::format(" %.6lg %.6lg", m_x, m_y); }

    private:
        float m_x;
        float m_y;
    };


} // namespace WebCore

#endif // ENABLE(SVG)
#endif
