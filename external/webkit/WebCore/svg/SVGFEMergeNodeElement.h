

#ifndef SVGFEMergeNodeElement_h
#define SVGFEMergeNodeElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGElement.h"
#include "SVGNames.h"

namespace WebCore {

    class SVGFEMergeNodeElement : public SVGElement {
    public:
        SVGFEMergeNodeElement(const QualifiedName&, Document*);
        virtual ~SVGFEMergeNodeElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGFEMergeNodeElement, SVGNames::inAttr, String, In1, in1)
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
