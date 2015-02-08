

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "CanvasObject.h"
#include "WebGLRenderingContext.h"

namespace WebCore {
    
CanvasObject::CanvasObject(WebGLRenderingContext* context)
    : m_object(0)
    , m_shouldDeleteObject(true)
    , m_context(context)
{
}

CanvasObject::~CanvasObject()
{
    if (m_context)
        m_context->removeObject(this);
}

void CanvasObject::setObject(Platform3DObject object, bool shouldDeleteObject)
{
    if (object == m_object)
        return;
        
    deleteObject();
    m_object = object;
    m_shouldDeleteObject = shouldDeleteObject;
}

void CanvasObject::deleteObject()
{
    if (m_object) {
        if (m_shouldDeleteObject)
            if (m_context) {
                m_context->graphicsContext3D()->makeContextCurrent();
                _deleteObject(m_object);
            }
        m_object = 0;
    }
    m_shouldDeleteObject = true;
}

}

#endif // ENABLE(3D_CANVAS)
