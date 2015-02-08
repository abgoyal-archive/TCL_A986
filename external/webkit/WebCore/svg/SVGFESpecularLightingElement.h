

#ifndef SVGFESpecularLightingElement_h
#define SVGFESpecularLightingElement_h

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFESpecularLighting.h"
#include "SVGFilterPrimitiveStandardAttributes.h"

namespace WebCore {

    extern char SVGKernelUnitLengthXIdentifier[];
    extern char SVGKernelUnitLengthYIdentifier[];

    class SVGFESpecularLightingElement : public SVGFilterPrimitiveStandardAttributes {
    public:
        SVGFESpecularLightingElement(const QualifiedName&, Document*);
        virtual ~SVGFESpecularLightingElement();
        
        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);
        virtual bool build(SVGResourceFilter*);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGFESpecularLightingElement, SVGNames::inAttr, String, In1, in1)
        DECLARE_ANIMATED_PROPERTY(SVGFESpecularLightingElement, SVGNames::specularConstantAttr, float, SpecularConstant, specularConstant)
        DECLARE_ANIMATED_PROPERTY(SVGFESpecularLightingElement, SVGNames::specularExponentAttr, float, SpecularExponent, specularExponent)
        DECLARE_ANIMATED_PROPERTY(SVGFESpecularLightingElement, SVGNames::surfaceScaleAttr, float, SurfaceScale, surfaceScale)
        DECLARE_ANIMATED_PROPERTY_MULTIPLE_WRAPPERS(SVGFESpecularLightingElement, SVGNames::kernelUnitLengthAttr, SVGKernelUnitLengthXIdentifier, float, KernelUnitLengthX, kernelUnitLengthX)
        DECLARE_ANIMATED_PROPERTY_MULTIPLE_WRAPPERS(SVGFESpecularLightingElement, SVGNames::kernelUnitLengthAttr, SVGKernelUnitLengthYIdentifier, float, KernelUnitLengthY, kernelUnitLengthY)

        PassRefPtr<LightSource> findLights() const;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
