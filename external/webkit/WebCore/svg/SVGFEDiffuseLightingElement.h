

#ifndef SVGFEDiffuseLightingElement_h
#define SVGFEDiffuseLightingElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFELightElement.h"
#include "SVGFilterPrimitiveStandardAttributes.h"

namespace WebCore {

    extern char SVGKernelUnitLengthXIdentifier[];
    extern char SVGKernelUnitLengthYIdentifier[];

    class FEDiffuseLighting;
    class SVGColor;

    class SVGFEDiffuseLightingElement : public SVGFilterPrimitiveStandardAttributes {
    public:
        SVGFEDiffuseLightingElement(const QualifiedName&, Document*);
        virtual ~SVGFEDiffuseLightingElement();

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool build(SVGResourceFilter*);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGFEDiffuseLightingElement, SVGNames::inAttr, String, In1, in1)
        DECLARE_ANIMATED_PROPERTY(SVGFEDiffuseLightingElement, SVGNames::diffuseConstantAttr, float, DiffuseConstant, diffuseConstant)
        DECLARE_ANIMATED_PROPERTY(SVGFEDiffuseLightingElement, SVGNames::surfaceScaleAttr, float, SurfaceScale, surfaceScale)
        DECLARE_ANIMATED_PROPERTY_MULTIPLE_WRAPPERS(SVGFEDiffuseLightingElement, SVGNames::kernelUnitLengthAttr, SVGKernelUnitLengthXIdentifier, float, KernelUnitLengthX, kernelUnitLengthX)
        DECLARE_ANIMATED_PROPERTY_MULTIPLE_WRAPPERS(SVGFEDiffuseLightingElement, SVGNames::kernelUnitLengthAttr, SVGKernelUnitLengthYIdentifier, float, KernelUnitLengthY, kernelUnitLengthY)

        PassRefPtr<LightSource> findLights() const;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
