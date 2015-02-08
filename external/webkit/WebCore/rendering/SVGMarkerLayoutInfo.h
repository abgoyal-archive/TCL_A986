

#ifndef SVGMarkerLayoutInfo_h
#define SVGMarkerLayoutInfo_h

#if ENABLE(SVG)
#include "RenderObject.h"
#include "SVGMarkerData.h"
#include <wtf/Noncopyable.h>

namespace WebCore {

class Path;
class SVGResourceMarker;

struct MarkerLayout {
    MarkerLayout(SVGResourceMarker* markerObj = 0, AffineTransform matrixObj = AffineTransform())
        : marker(markerObj)
        , matrix(matrixObj)
    {
        ASSERT(marker);
    }

    SVGResourceMarker* marker;
    AffineTransform matrix;
};

class SVGMarkerLayoutInfo : public Noncopyable {
public:
    SVGMarkerLayoutInfo();
    ~SVGMarkerLayoutInfo();

    FloatRect calculateBoundaries(SVGResourceMarker* startMarker, SVGResourceMarker* midMarker, SVGResourceMarker* endMarker, float strokeWidth, const Path&);
    void drawMarkers(RenderObject::PaintInfo&);

    // Used by static inline helper functions in SVGMarkerLayoutInfo.cpp
    SVGMarkerData& markerData() { return m_markerData; }
    SVGResourceMarker* midMarker() const { return m_midMarker; }
    int& elementIndex() { return m_elementIndex; }
    void addLayoutedMarker(SVGResourceMarker*, const FloatPoint& origin, float angle);

private:
    SVGResourceMarker* m_midMarker;

    // Used while layouting markers
    int m_elementIndex;
    SVGMarkerData m_markerData;
    float m_strokeWidth;

    // Holds the final computed result
    Vector<MarkerLayout> m_layout;
};

}

#endif // ENABLE(SVG)
#endif // SVGMarkerLayoutInfo_h
