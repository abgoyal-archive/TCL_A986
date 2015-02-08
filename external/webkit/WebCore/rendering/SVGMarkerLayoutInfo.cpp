

#include "config.h"

#if ENABLE(SVG)
#include "SVGMarkerLayoutInfo.h"

#include "RenderSVGViewportContainer.h"
#include "SVGResourceMarker.h"

namespace WebCore {

SVGMarkerLayoutInfo::SVGMarkerLayoutInfo()
    : m_midMarker(0)
    , m_elementIndex(0)
    , m_strokeWidth(0)
{
}

SVGMarkerLayoutInfo::~SVGMarkerLayoutInfo()
{
}

static inline void processStartAndMidMarkers(void* infoPtr, const PathElement* element)
{
    SVGMarkerLayoutInfo& info = *reinterpret_cast<SVGMarkerLayoutInfo*>(infoPtr);
    SVGMarkerData& markerData = info.markerData();
    int& elementIndex = info.elementIndex();

    // First update the outslope for the previous element
    markerData.updateOutslope(element->points[0]);

    // Draw the marker for the previous element
    SVGResourceMarker* marker = markerData.marker();
    if (elementIndex > 0 && marker)
        info.addLayoutedMarker(marker, markerData.origin(), markerData.currentAngle());

    // Update our marker data for this element
    markerData.updateMarkerDataForPathElement(element);

    // After drawing the start marker, switch to drawing mid markers
    if (elementIndex == 1)
        markerData.updateTypeAndMarker(SVGMarkerData::Mid, info.midMarker());

    ++elementIndex;
}

FloatRect SVGMarkerLayoutInfo::calculateBoundaries(SVGResourceMarker* startMarker, SVGResourceMarker* midMarker, SVGResourceMarker* endMarker, float strokeWidth, const Path& path)
{
    m_layout.clear();
    m_midMarker = midMarker;
    m_strokeWidth = strokeWidth;
    m_elementIndex = 0;
    m_markerData = SVGMarkerData(SVGMarkerData::Start, startMarker);
    path.apply(this, processStartAndMidMarkers);

    if (endMarker) {
        m_markerData.updateTypeAndMarker(SVGMarkerData::End, endMarker);
        addLayoutedMarker(endMarker, m_markerData.origin(), m_markerData.currentAngle());
    }

    if (m_layout.isEmpty())
        return FloatRect();

    Vector<MarkerLayout>::iterator it = m_layout.begin();
    Vector<MarkerLayout>::iterator end = m_layout.end();

    FloatRect bounds;
    for (; it != end; ++it) {
        MarkerLayout& layout = *it;

        RenderSVGViewportContainer* markerContent = layout.marker->renderer();
        ASSERT(markerContent);

        bounds.unite(markerContent->markerBoundaries(layout.matrix));
    }

    return bounds;
}

void SVGMarkerLayoutInfo::drawMarkers(RenderObject::PaintInfo& paintInfo)
{
    if (m_layout.isEmpty())
        return;

    Vector<MarkerLayout>::iterator it = m_layout.begin();
    Vector<MarkerLayout>::iterator end = m_layout.end();

    for (; it != end; ++it) {
        MarkerLayout& layout = *it;
        layout.marker->draw(paintInfo, layout.matrix);
    }
}

void SVGMarkerLayoutInfo::addLayoutedMarker(SVGResourceMarker* marker, const FloatPoint& origin, float angle)
{
    ASSERT(marker);
    m_layout.append(MarkerLayout(marker, marker->markerTransformation(origin, angle, m_strokeWidth)));
}

}

#endif // ENABLE(SVG)
