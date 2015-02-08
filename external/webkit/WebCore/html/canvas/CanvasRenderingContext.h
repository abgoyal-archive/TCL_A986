

#ifndef CanvasRenderingContext_h
#define CanvasRenderingContext_h

#include <wtf/Noncopyable.h>

namespace WebCore {

    class CanvasObject;
    class HTMLCanvasElement;

    class CanvasRenderingContext : public Noncopyable {
    public:
        CanvasRenderingContext(HTMLCanvasElement*);
        virtual ~CanvasRenderingContext() { }
        
        // Ref and deref the m_canvas
        void ref();
        void deref();
        
        HTMLCanvasElement* canvas() const { return m_canvas; }
        
        virtual bool is2d() const { return false; }
        virtual bool is3d() const { return false; }

    private:
        HTMLCanvasElement* m_canvas;
    };

} // namespace WebCore

#endif
