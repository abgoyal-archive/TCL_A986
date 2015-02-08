

#ifndef SVGPaintServerSolid_h
#define SVGPaintServerSolid_h

#if ENABLE(SVG)

#include "Color.h"
#include "SVGPaintServer.h"

namespace WebCore {

    class SVGPaintServerSolid : public SVGPaintServer {
    public:
        static PassRefPtr<SVGPaintServerSolid> create() { return adoptRef(new SVGPaintServerSolid); }
        virtual ~SVGPaintServerSolid();

        virtual SVGPaintServerType type() const { return SolidPaintServer; }

        Color color() const;
        void setColor(const Color&);

        virtual TextStream& externalRepresentation(TextStream&) const;

        virtual bool setup(GraphicsContext*&, const RenderObject*, const RenderStyle*, SVGPaintTargetType, bool isPaintingText) const;

    private:
        SVGPaintServerSolid();

        Color m_color;
    };

} // namespace WebCore

#endif

#endif // SVGPaintServerSolid_h
