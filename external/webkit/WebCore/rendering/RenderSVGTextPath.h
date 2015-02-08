

#ifndef RenderSVGTextPath_h
#define RenderSVGTextPath_h

#if ENABLE(SVG)
#include "RenderSVGInline.h"

namespace WebCore {

    class RenderSVGTextPath : public RenderSVGInline {
    public:
        RenderSVGTextPath(Node*);

        Path layoutPath() const;
        float startOffset() const;
        bool exactAlignment() const;
        bool stretchMethod() const;

    private:
        virtual const char* renderName() const { return "RenderSVGTextPath"; }

        float m_startOffset;

        bool m_exactAlignment : 1;
        bool m_stretchMethod : 1;

        Path m_layoutPath;
    };

    inline RenderSVGTextPath* toRenderSVGTextPath(RenderObject* object)
    { 
        ASSERT(!object || !strcmp(object->renderName(), "RenderSVGTextPath"));
        return static_cast<RenderSVGTextPath*>(object);
    }

    // This will catch anyone doing an unnecessary cast.
    void toRenderSVGTextPath(const RenderSVGTextPath*);

}

#endif // ENABLE(SVG)
#endif // RenderSVGTextPath_h
