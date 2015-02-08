

#ifndef SVGPaintServerPattern_h
#define SVGPaintServerPattern_h

#if ENABLE(SVG)

#include "AffineTransform.h"
#include "FloatRect.h"
#include "Pattern.h"
#include "SVGPaintServer.h"

#include <memory>

#include <wtf/OwnPtr.h>
#include <wtf/PassOwnPtr.h>

namespace WebCore {

    class GraphicsContext;
    class ImageBuffer;
    class SVGPatternElement;

    class SVGPaintServerPattern : public SVGPaintServer {
    public:
        static PassRefPtr<SVGPaintServerPattern> create(const SVGPatternElement* owner) { return adoptRef(new SVGPaintServerPattern(owner)); }

        virtual ~SVGPaintServerPattern();

        virtual SVGPaintServerType type() const { return PatternPaintServer; }

        // Pattern boundaries
        void setPatternBoundaries(const FloatRect&);
        FloatRect patternBoundaries() const;

        ImageBuffer* tile() const;
        void setTile(PassOwnPtr<ImageBuffer>);

        AffineTransform patternTransform() const;
        void setPatternTransform(const AffineTransform&);

        virtual TextStream& externalRepresentation(TextStream&) const;

        virtual bool setup(GraphicsContext*&, const RenderObject*, const RenderStyle*, SVGPaintTargetType, bool isPaintingText) const;
        virtual void teardown(GraphicsContext*&, const RenderObject*, SVGPaintTargetType, bool isPaintingText) const;

    private:
        SVGPaintServerPattern(const SVGPatternElement*);
        
        OwnPtr<ImageBuffer> m_tile;
        const SVGPatternElement* m_ownerElement;
        AffineTransform m_patternTransform;
        FloatRect m_patternBoundaries;

        mutable RefPtr<Pattern> m_pattern;
    };

} // namespace WebCore

#endif

#endif // SVGPaintServerPattern_h
