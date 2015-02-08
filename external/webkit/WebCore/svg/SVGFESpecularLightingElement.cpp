

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFESpecularLightingElement.h"

#include "MappedAttribute.h"
#include "RenderObject.h"
#include "SVGColor.h"
#include "SVGFELightElement.h"
#include "SVGNames.h"
#include "SVGParserUtilities.h"
#include "SVGResourceFilter.h"

namespace WebCore {

SVGFESpecularLightingElement::SVGFESpecularLightingElement(const QualifiedName& tagName, Document* doc)
    : SVGFilterPrimitiveStandardAttributes(tagName, doc)
    , m_specularConstant(1.0f)
    , m_specularExponent(1.0f)
    , m_surfaceScale(1.0f)
{
}

SVGFESpecularLightingElement::~SVGFESpecularLightingElement()
{
}

void SVGFESpecularLightingElement::parseMappedAttribute(MappedAttribute* attr)
{    
    const String& value = attr->value();
    if (attr->name() == SVGNames::inAttr)
        setIn1BaseValue(value);
    else if (attr->name() == SVGNames::surfaceScaleAttr)
        setSurfaceScaleBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::specularConstantAttr)
        setSpecularConstantBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::specularExponentAttr)
        setSpecularExponentBaseValue(value.toFloat());
    else if (attr->name() == SVGNames::kernelUnitLengthAttr) {
        float x, y;
        if (parseNumberOptionalNumber(value, x, y)) {
            setKernelUnitLengthXBaseValue(x);
            setKernelUnitLengthYBaseValue(y);
        }
    } else
        SVGFilterPrimitiveStandardAttributes::parseMappedAttribute(attr);
}

void SVGFESpecularLightingElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGFilterPrimitiveStandardAttributes::synchronizeProperty(attrName);

    if (attrName == anyQName()) {
        synchronizeIn1();
        synchronizeSurfaceScale();
        synchronizeSpecularConstant();
        synchronizeSpecularExponent();
        synchronizeKernelUnitLengthX();
        synchronizeKernelUnitLengthY();
        return;
    }

    if (attrName == SVGNames::inAttr)
        synchronizeIn1();
    else if (attrName == SVGNames::surfaceScaleAttr)
        synchronizeSurfaceScale();
    else if (attrName == SVGNames::specularConstantAttr)
        synchronizeSpecularConstant();
    else if (attrName == SVGNames::specularExponentAttr)
        synchronizeSpecularExponent();
    else if (attrName == SVGNames::kernelUnitLengthAttr) {
        synchronizeKernelUnitLengthX();
        synchronizeKernelUnitLengthY();
    }
}

PassRefPtr<LightSource> SVGFESpecularLightingElement::findLights() const
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

bool SVGFESpecularLightingElement::build(SVGResourceFilter* filterResource)
{
    FilterEffect* input1 = filterResource->builder()->getEffectById(in1());
    
    if (!input1)
        return false;
    
    RefPtr<RenderStyle> filterStyle = styleForRenderer();    
    
    Color color = filterStyle->svgStyle()->lightingColor();
    
    RefPtr<FilterEffect> effect = FESpecularLighting::create(input1, color, surfaceScale(), specularConstant(), 
                                        specularExponent(), kernelUnitLengthX(), kernelUnitLengthY(), findLights());
    filterResource->addFilterEffect(this, effect.release());

    return true;
}

}

#endif // ENABLE(SVG)
