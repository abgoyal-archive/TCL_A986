

#include "config.h"
#if ENABLE(SVG)
#include "SVGDocument.h"

#include "EventNames.h"
#include "ExceptionCode.h"
#include "FrameView.h"
#include "RenderView.h"
#include "SVGElement.h"
#include "SVGNames.h"
#include "SVGSVGElement.h"
#include "SVGViewSpec.h"
#include "SVGZoomEvent.h"
#include "SVGZoomAndPan.h"

namespace WebCore {

SVGDocument::SVGDocument(Frame* frame)
    : Document(frame, false, false)
{
}

SVGDocument::~SVGDocument()
{
}

SVGSVGElement* SVGDocument::rootElement() const
{
    Element* elem = documentElement();
    if (elem && elem->hasTagName(SVGNames::svgTag))
        return static_cast<SVGSVGElement*>(elem);

    return 0;
}

void SVGDocument::dispatchZoomEvent(float prevScale, float newScale)
{
    ExceptionCode ec = 0;
    RefPtr<SVGZoomEvent> event = static_pointer_cast<SVGZoomEvent>(createEvent("SVGZoomEvents", ec));
    event->initEvent(eventNames().zoomEvent, true, false);
    event->setPreviousScale(prevScale);
    event->setNewScale(newScale);
    rootElement()->dispatchEvent(event.release(), ec);
}

void SVGDocument::dispatchScrollEvent()
{
    ExceptionCode ec = 0;
    RefPtr<Event> event = createEvent("SVGEvents", ec);
    event->initEvent(eventNames().scrollEvent, true, false);
    rootElement()->dispatchEvent(event.release(), ec);
}

bool SVGDocument::zoomAndPanEnabled() const
{
    if (rootElement()) {
        if (rootElement()->useCurrentView()) {
            if (rootElement()->currentView())
                return rootElement()->currentView()->zoomAndPan() == SVGZoomAndPan::SVG_ZOOMANDPAN_MAGNIFY;
        } else
            return rootElement()->zoomAndPan() == SVGZoomAndPan::SVG_ZOOMANDPAN_MAGNIFY;
    }

    return false;
}

void SVGDocument::startPan(const FloatPoint& start)
{
    if (rootElement())
        m_translate = FloatPoint(start.x() - rootElement()->currentTranslate().x(), rootElement()->currentTranslate().y() + start.y());
}

void SVGDocument::updatePan(const FloatPoint& pos) const
{
    if (rootElement()) {
        rootElement()->setCurrentTranslate(FloatPoint(pos.x() - m_translate.x(), m_translate.y() - pos.y()));
        if (renderer())
            renderer()->repaint();
    }
}

bool SVGDocument::childShouldCreateRenderer(Node* node) const
{
    if (node->hasTagName(SVGNames::svgTag))
        return static_cast<SVGSVGElement*>(node)->isValid();
    return true;
}

}

// vim:ts=4:noet
#endif // ENABLE(SVG)
