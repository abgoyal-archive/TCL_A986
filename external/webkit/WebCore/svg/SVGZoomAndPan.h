

#ifndef SVGZoomAndPan_h
#define SVGZoomAndPan_h

#if ENABLE(SVG)
#include "PlatformString.h"

namespace WebCore {

    class MappedAttribute;
    class QualifiedName;

    class SVGZoomAndPan {
    public:
        enum SVGZoomAndPanType {
            SVG_ZOOMANDPAN_UNKNOWN = 0,
            SVG_ZOOMANDPAN_DISABLE = 1,
            SVG_ZOOMANDPAN_MAGNIFY = 2
        };

        SVGZoomAndPan();
        virtual ~SVGZoomAndPan();

        unsigned short zoomAndPan() const;
        virtual void setZoomAndPan(unsigned short zoomAndPan);

        bool parseMappedAttribute(MappedAttribute*);
        bool isKnownAttribute(const QualifiedName&);

        bool parseZoomAndPan(const UChar*& start, const UChar* end);

    private:
        unsigned short m_zoomAndPan;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGZoomAndPan_h
