

#ifndef SVGStopElement_h
#define SVGStopElement_h

#if ENABLE(SVG)
#include "SVGNames.h"
#include "SVGStyledElement.h"

namespace WebCore {

    class SVGStopElement : public SVGStyledElement {
    public:
        SVGStopElement(const QualifiedName&, Document*);
        virtual ~SVGStopElement();

        virtual void synchronizeProperty(const QualifiedName&);

    private:
        virtual bool isGradientStop() const { return true; }

        virtual void parseMappedAttribute(MappedAttribute*);

        virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);

        DECLARE_ANIMATED_PROPERTY(SVGStopElement, SVGNames::offsetAttr, float, Offset, offset)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
