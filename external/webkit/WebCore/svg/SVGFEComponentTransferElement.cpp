

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEComponentTransferElement.h"

#include "Attr.h"
#include "MappedAttribute.h"
#include "SVGFEFuncAElement.h"
#include "SVGFEFuncBElement.h"
#include "SVGFEFuncGElement.h"
#include "SVGFEFuncRElement.h"
#include "SVGNames.h"
#include "SVGRenderStyle.h"
#include "SVGResourceFilter.h"

namespace WebCore {

SVGFEComponentTransferElement::SVGFEComponentTransferElement(const QualifiedName& tagName, Document* doc)
    : SVGFilterPrimitiveStandardAttributes(tagName, doc)
{
}

SVGFEComponentTransferElement::~SVGFEComponentTransferElement()
{
}

void SVGFEComponentTransferElement::parseMappedAttribute(MappedAttribute* attr)
{
    const String& value = attr->value();
    if (attr->name() == SVGNames::inAttr)
        setIn1BaseValue(value);
    else
        SVGFilterPrimitiveStandardAttributes::parseMappedAttribute(attr);
}

void SVGFEComponentTransferElement::synchronizeProperty(const QualifiedName& attrName)
{
    SVGFilterPrimitiveStandardAttributes::synchronizeProperty(attrName);

    if (attrName == anyQName() || attrName == SVGNames::inAttr)
        synchronizeIn1();
}

bool SVGFEComponentTransferElement::build(SVGResourceFilter* filterResource)
{
    FilterEffect* input1 = filterResource->builder()->getEffectById(in1());
    
    if (!input1)
        return false;

    ComponentTransferFunction red;
    ComponentTransferFunction green;
    ComponentTransferFunction blue;
    ComponentTransferFunction alpha;
    
    for (Node* n = firstChild(); n != 0; n = n->nextSibling()) {
        if (n->hasTagName(SVGNames::feFuncRTag))
            red = static_cast<SVGFEFuncRElement*>(n)->transferFunction();
        else if (n->hasTagName(SVGNames::feFuncGTag))
            green = static_cast<SVGFEFuncGElement*>(n)->transferFunction();
        else if (n->hasTagName(SVGNames::feFuncBTag))
           blue = static_cast<SVGFEFuncBElement*>(n)->transferFunction();
        else if (n->hasTagName(SVGNames::feFuncATag))
            alpha = static_cast<SVGFEFuncAElement*>(n)->transferFunction();
    }
    
    RefPtr<FilterEffect> effect = FEComponentTransfer::create(input1, red, green, blue, alpha);
    filterResource->addFilterEffect(this, effect.release());
    
    return true;
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
