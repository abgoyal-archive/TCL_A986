

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEDiffuseLightingElement.h"

#include "Attr.h"
#include "MappedAttribute.h"
#include "RenderObject.h"
#include "SVGColor.h"
#include "SVGFEDiffuseLighting.h"
#include "SVGFELightElement.h"
#include "SVGNames.h"
#include "SVGParserUtilities.h"
#include "SVGRenderStyle.h"
#include "SVGResourceFilter.h"

namespace WebCore {

char SVGKernelUnitLengthXIdentifier[] = "SVGKernelUnitLengthX";
char SVGKernelUnitLengthYIdentifier[] = "SVGKernelUnitLengthY";

SVGFEDiffuseLightingElement::SVGFEDiffuseLightingElement(const QualifiedName& tagName, Document* doc)
    : SVGFilterPrimitiveStandardAttributes(tagName, doc)
    , m_diffuseConstant(1.0f)
    , m_surfaceScale(1.0f)
{
}

SVGFEDiffuseLightingElement::~SVGFEDiffuseLightingElement()
{
}

void SVGFEDiffuseLightingElement::parseMappedAttribute(MappedAttribute *attr)
{
    const String& value = attr->value();
    if (attr->name() == SVGNames::inAttr)
        setIn1BaseValue(value);
    else if (attr->name() == SVGNames::surfaceScaleAttr)
        setSurfaceScaleBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::diffuseConstantAttr)
        setDiffuseConstantBaseValue(value.toInt());
    else if (attr->name() == SVGNames::kernelUnitLengthAttr) {
        float x, y;
        if (parseNumberOptionalNumber(value, x, y)) {
            setKernelUnitLengthXBaseValue(x);
            setKernelUnitLengthYBaseValue(y);
        }
    } else
        SVGFilterPrimitiveStandardAttributes::parseMappedAttribute(attr);
}

void SVGFEDiffuseLightingElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGFilterPrimitiveStandardAttributes::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeIn1();
        synchronizeSurfaceScale();
        synchronizeDiffuseConstant();
        synchronizeKernelUnitLengthX();
        synchronizeKernelUnitLengthY();
        return;
    }

    if (attrName == SVGNames::inAttr)
        synchronizeIn1();
    else if (attrName == SVGNames::surfaceScaleAttr)
        synchronizeSurfaceScale();
    else if (attrName == SVGNames::diffuseConstantAttr)
        synchronizeDiffuseConstant();
    else if (attrName == SVGNames::kernelUnitLengthAttr) {
        synchronizeKernelUnitLengthX();
        synchronizeKernelUnitLengthY();
    }
}

bool SVGFEDiffuseLightingElement::build(SVGResourceFilter* filterResource)
{
    FilterEffect* input1 = filterResource->builder()->getEffectById(in1());
    
    if (!input1)
        return false;
    
    RefPtr<RenderStyle> filterStyle = styleForRenderer();
    Color color = filterStyle->svgStyle()->lightingColor();
    
    RefPtr<FilterEffect> effect = FEDiffuseLighting::create(input1, color, surfaceScale(), diffuseConstant(), 
                                            kernelUnitLengthX(), kernelUnitLengthY(), findLights());
    filterResource->addFilterEffect(this, effect.release());
    
    return true;
}

PassRefPtr<LightSource> SVGFEDiffuseLightingElement::findLights() const
{
    for (Node* n = firstChild(); n; n = n->nextSibling()) {
        if (n->hasTagName(SVGNames::feDistantLightTag) ||
            n->hasTagName(SVGNames::fePointLightTag) ||
            n->hasTagName(SVGNames::feSpotLightTag)) {
            SVGFELightElement* lightNode = static_cast<SVGFELightElement*>(n); 
            return lightNode->lightSource();
        }
    }

    return 0;
}

}

#endif // ENABLE(SVG)
