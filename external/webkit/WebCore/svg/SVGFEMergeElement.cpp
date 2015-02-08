

#include "config.h"

#if ENABLE(SVG) && ENABLE(FILTERS)
#include "SVGFEMergeElement.h"

#include "SVGFEMergeNodeElement.h"
#include "SVGResourceFilter.h"

namespace WebCore {

SVGFEMergeElement::SVGFEMergeElement(const QualifiedName& tagName, Document* doc)
    : SVGFilterPrimitiveStandardAttributes(tagName, doc)
{
}

SVGFEMergeElement::~SVGFEMergeElement()
{
}

bool SVGFEMergeElement::build(SVGResourceFilter* filterResource)
{
    Vector<RefPtr<FilterEffect> > mergeInputs;
    for (Node* n = firstChild(); n != 0; n = n->nextSibling()) {
        if (n->hasTagName(SVGNames::feMergeNodeTag)) {
            FilterEffect* mergeEffect = filterResource->builder()->getEffectById(static_cast<SVGFEMergeNodeElement*>(n)->in1());
            if (!mergeEffect)
                return false;
            mergeInputs.append(mergeEffect);
        }
    }

    if (mergeInputs.isEmpty())
        return false;

    RefPtr<FilterEffect> effect = FEMerge::create(mergeInputs);
    filterResource->addFilterEffect(this, effect.release());

    return true;
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
