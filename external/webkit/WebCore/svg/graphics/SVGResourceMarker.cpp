

#include "config.h"

#if ENABLE(SVG)
#include "SVGResourceMarker.h"

#include "AffineTransform.h"
#include "GraphicsContext.h"
#include "RenderSVGViewportContainer.h"
#include "TextStream.h"
#include <wtf/StdLibExtras.h>

namespace WebCore {

SVGResourceMarker::SVGResourceMarker()
    : SVGResource()
    , m_angle(-1) // just like using setAutoAngle()
    , m_renderer(0)
    , m_useStrokeWidth(true)
{
}

SVGResourceMarker::~SVGResourceMarker()
{
}

AffineTransform SVGResourceMarker::markerTransformation(const FloatPoint& origin, float angle, float strokeWidth) const
{
    ASSERT(m_renderer);

    AffineTransform transform;
    transform.translate(origin.x(), origin.y());
    transform.rotate(m_angle == -1 ? angle : m_angle);
    transform = m_renderer->markerContentTransformation(transform, m_referencePoint, m_useStrokeWidth ? strokeWidth : -1);
    return transform;
}

void SVGResourceMarker::draw(RenderObject::PaintInfo& paintInfo, const AffineTransform& transform)
{
    if (!m_renderer)
        return;

    DEFINE_STATIC_LOCAL(HashSet<SVGResourceMarker*>, currentlyDrawingMarkers, ());

    // avoid drawing circular marker references
    if (currentlyDrawingMarkers.contains(this))
        return;

    currentlyDrawingMarkers.add(this);
    ASSERT(!m_renderer->drawsContents());
    RenderObject::PaintInfo info(paintInfo);
    info.context->save();
    applyTransformToPaintInfo(info, transform);
    m_renderer->setDrawsContents(true);
    m_renderer->paint(info, 0, 0);
    m_renderer->setDrawsContents(false);
    info.context->restore();

    currentlyDrawingMarkers.remove(this);
}

TextStream& SVGResourceMarker::externalRepresentation(TextStream& ts) const
{
    ts << "[type=MARKER]"
        << " [angle=";

    if (angle() == -1)
        ts << "auto" << "]";
    else
        ts << angle() << "]";

    ts << " [ref x=" << m_referencePoint.x() << " y=" << m_referencePoint.y() << "]";
    return ts;
}

SVGResourceMarker* getMarkerById(Document* document, const AtomicString& id, const RenderObject* object)
{
    SVGResource* resource = getResourceById(document, id, object);
    if (resource && resource->isMarker())
        return static_cast<SVGResourceMarker*>(resource);

    return 0;
}

} // namespace WebCore

#endif
