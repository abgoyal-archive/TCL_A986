

#ifndef SVGPathSegClosePath_h
#define SVGPathSegClosePath_h

#if ENABLE(SVG)

#include "SVGPathSeg.h"

namespace WebCore {
    class SVGPathSegClosePath : public SVGPathSeg {
    public:
        static PassRefPtr<SVGPathSegClosePath> create() { return adoptRef(new SVGPathSegClosePath); }
        virtual ~SVGPathSegClosePath();

        virtual unsigned short pathSegType() const { return PATHSEG_CLOSEPATH; }
        virtual String pathSegTypeAsLetter() const { return "Z"; }
        virtual String toString() const { return "Z"; }
        
    private:
        SVGPathSegClosePath();
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif

// vim:ts=4:noet
