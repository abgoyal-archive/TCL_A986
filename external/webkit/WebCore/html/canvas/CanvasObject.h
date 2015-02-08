

#ifndef CanvasObject_h
#define CanvasObject_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

#include "GraphicsContext3D.h"

namespace WebCore {

    class WebGLRenderingContext;
    
    class CanvasObject : public RefCounted<CanvasObject> {
    public:
        virtual ~CanvasObject();
        
        Platform3DObject object() const { return m_object; }
        void setObject(Platform3DObject, bool shouldDeleteObject = true);
        void deleteObject();
        
        void detachContext()
        {
            deleteObject();
            m_context = 0;
        }

        WebGLRenderingContext* context() const { return m_context; }

    protected:
        CanvasObject(WebGLRenderingContext*);
        virtual void _deleteObject(Platform3DObject) = 0;
    
    private:
        Platform3DObject m_object;
        // The shouldDeleteObject flag indicates whether this wrapper
        // owns the underlying resource and should delete it when the
        // wrapper is unreferenced for the last time and deleted. It
        // is only set to false for certain objects returned from get
        // queries. FIXME: should consider canonicalizing all of these
        // objects in the future.
        bool m_shouldDeleteObject;
        WebGLRenderingContext* m_context;
    };
    
} // namespace WebCore

#endif // CanvasObject_h
