

#include "config.h"
#include "CanvasRenderingContext.h"

#include "HTMLCanvasElement.h"

namespace WebCore {

CanvasRenderingContext::CanvasRenderingContext(HTMLCanvasElement* canvas)
    : m_canvas(canvas)
{
}

void CanvasRenderingContext::ref()
{
    m_canvas->ref();
}

void CanvasRenderingContext::deref()
{
    m_canvas->deref(); 
}

} // namespace WebCore
