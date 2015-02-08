

#include "config.h"
#include "CSSCanvasValue.h"

#include "ImageBuffer.h"
#include "RenderObject.h"

namespace WebCore {

CSSCanvasValue::~CSSCanvasValue()
{
    if (m_element)
        m_element->setObserver(0);
}

String CSSCanvasValue::cssText() const
{
    String result = "-webkit-canvas(";
    result += m_name  + ")";
    return result;
}

void CSSCanvasValue::canvasChanged(HTMLCanvasElement*, const FloatRect& changedRect)
{
    IntRect imageChangeRect = enclosingIntRect(changedRect);
    RenderObjectSizeCountMap::const_iterator end = m_clients.end();
    for (RenderObjectSizeCountMap::const_iterator curr = m_clients.begin(); curr != end; ++curr)
        curr->first->imageChanged(static_cast<WrappedImagePtr>(this), &imageChangeRect);
}

void CSSCanvasValue::canvasResized(HTMLCanvasElement*)
{
    RenderObjectSizeCountMap::const_iterator end = m_clients.end();
    for (RenderObjectSizeCountMap::const_iterator curr = m_clients.begin(); curr != end; ++curr)
        curr->first->imageChanged(static_cast<WrappedImagePtr>(this));
}

void CSSCanvasValue::canvasDestroyed(HTMLCanvasElement* element)
{
    ASSERT(element == m_element);
    if (element == m_element)
        m_element = 0;
}

IntSize CSSCanvasValue::fixedSize(const RenderObject* renderer)
{
    if (HTMLCanvasElement* elt = element(renderer->document()))
        return IntSize(elt->width(), elt->height());
    return IntSize();
}

HTMLCanvasElement* CSSCanvasValue::element(Document* document)
{
     if (!m_element) {
        m_element = document->getCSSCanvasElement(m_name);
        if (!m_element)
            return 0;
        m_element->setObserver(this);
    }
    return m_element;
}

Image* CSSCanvasValue::image(RenderObject* renderer, const IntSize& /*size*/)
{
    ASSERT(m_clients.contains(renderer));
    HTMLCanvasElement* elt = element(renderer->document());
    if (!elt || !elt->buffer())
        return 0;
    return elt->buffer()->image();
}

} // namespace WebCore
