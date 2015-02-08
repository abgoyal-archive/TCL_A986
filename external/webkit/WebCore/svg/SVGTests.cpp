

#include "config.h"

#if ENABLE(SVG)
#include "SVGTests.h"

#include "DOMImplementation.h"
#include "Language.h"
#include "MappedAttribute.h"
#include "SVGElement.h"
#include "SVGNames.h"
#include "SVGStringList.h"

namespace WebCore {

SVGTests::SVGTests()
{
}

SVGTests::~SVGTests()
{
}

SVGStringList* SVGTests::requiredFeatures() const
{
    if (!m_features)
        m_features = SVGStringList::create(SVGNames::requiredFeaturesAttr);

    return m_features.get();
}

SVGStringList* SVGTests::requiredExtensions() const
{
    if (!m_extensions)
        m_extensions = SVGStringList::create(SVGNames::requiredExtensionsAttr);

    return m_extensions.get();
}

SVGStringList* SVGTests::systemLanguage() const
{
    if (!m_systemLanguage)
        m_systemLanguage = SVGStringList::create(SVGNames::systemLanguageAttr);

    return m_systemLanguage.get();
}

bool SVGTests::hasExtension(const String&) const
{
    return false;
}

bool SVGTests::isValid() const
{
    ExceptionCode ec = 0;

    if (m_features) {
        for (unsigned long i = 0; i < m_features->numberOfItems(); i++) {
            String value = m_features->getItem(i, ec);
            if (value.isEmpty() || !DOMImplementation::hasFeature(value, String()))
                return false;
        }
    }

    if (m_systemLanguage) {
        for (unsigned long i = 0; i < m_systemLanguage->numberOfItems(); i++)
            if (m_systemLanguage->getItem(i, ec) != defaultLanguage().substring(0, 2))
                return false;
    }

    if (m_extensions && m_extensions->numberOfItems() > 0)
        return false;

    return true;
}

bool SVGTests::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == SVGNames::requiredFeaturesAttr) {
        requiredFeatures()->reset(attr->value());
        return true;
    } else if (attr->name() == SVGNames::requiredExtensionsAttr) {
        requiredExtensions()->reset(attr->value());
        return true;
    } else if (attr->name() == SVGNames::systemLanguageAttr) {
        systemLanguage()->reset(attr->value());
        return true;
    }
    
    return false;
}

bool SVGTests::isKnownAttribute(const QualifiedName& attrName)
{
    return (attrName == SVGNames::requiredFeaturesAttr ||
            attrName == SVGNames::requiredExtensionsAttr ||
            attrName == SVGNames::systemLanguageAttr);
}

}

#endif // ENABLE(SVG)
