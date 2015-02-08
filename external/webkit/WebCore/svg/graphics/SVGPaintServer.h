

#ifndef SVGPaintServer_h
#define SVGPaintServer_h

#if ENABLE(SVG)

#include "DashArray.h"
#include "RenderObject.h"
#include "SVGResource.h"

#if PLATFORM(CG)
#include <ApplicationServices/ApplicationServices.h>
#endif

namespace WebCore {

    enum SVGPaintServerType {
        // Painting mode
        SolidPaintServer = 0,
        PatternPaintServer = 1,
        LinearGradientPaintServer = 2,
        RadialGradientPaintServer = 3
    };

    enum SVGPaintTargetType {
        // Target mode
        ApplyToFillTargetType = 1,
        ApplyToStrokeTargetType = 2
    };

    class GraphicsContext;
    class RenderObject;
    class RenderStyle;
    class SVGPaintServerSolid;

    class SVGPaintServer : public SVGResource {
    public:
        virtual ~SVGPaintServer();

        virtual SVGResourceType resourceType() const { return PaintServerResourceType; }

        virtual SVGPaintServerType type() const = 0;
        virtual TextStream& externalRepresentation(TextStream&) const = 0;

        // To be implemented in platform specific code.
        virtual void draw(GraphicsContext*&, const RenderObject*, SVGPaintTargetType) const;
        virtual void teardown(GraphicsContext*&, const RenderObject*, SVGPaintTargetType, bool isPaintingText = false) const;
        virtual void renderPath(GraphicsContext*&, const RenderObject*, SVGPaintTargetType) const;

        virtual bool setup(GraphicsContext*&, const RenderObject*, const RenderStyle*, SVGPaintTargetType, bool isPaintingText = false) const = 0;

        bool setup(GraphicsContext*&, const RenderObject*, SVGPaintTargetType, bool isPaintingText = false) const;

        static SVGPaintServer* strokePaintServer(const RenderStyle*, const RenderObject*);
        static SVGPaintServer* fillPaintServer(const RenderStyle*, const RenderObject*);
        static SVGPaintServerSolid* sharedSolidPaintServer();

    protected:
        SVGPaintServer();        
    };

    TextStream& operator<<(TextStream&, const SVGPaintServer&);

    SVGPaintServer* getPaintServerById(Document*, const AtomicString&, const RenderObject*);

    void applyStrokeStyleToContext(GraphicsContext*, const RenderStyle*, const RenderObject*);
    DashArray dashArrayFromRenderingStyle(const RenderStyle* style, RenderStyle* rootStyle);
} // namespace WebCore

#endif

#endif // SVGPaintServer_h
