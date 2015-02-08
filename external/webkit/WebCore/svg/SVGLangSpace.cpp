

#include "config.h"

#if ENABLE(SVG)
#include "SVGLangSpace.h"

#include "MappedAttribute.h"
#include "SVGElement.h"
#include "XMLNames.h"
#include <wtf/StdLibExtras.h>

namespace WebCore {

SVGLangSpace::SVGLangSpace()
{
}

SVGLangSpace::~SVGLangSpace()
{
}

const AtomicString& SVGLangSpace::xmllang() const
{
    return m_lang;
}

void SVGLangSpace::setXmllang(const AtomicString& xmlLang)
{
    m_lang = xmlLang;
}

const AtomicString& SVGLangSpace::xmlspace() const
{
    if (!m_space) {
        DEFINE_STATIC_LOCAL(const AtomicString, defaultString, ("default"));
        return defaultString;
    }

    return m_space;
}

void SVGLangSpace::setXmlspace(const AtomicString& xmlSpace)
{
    m_space = xmlSpace;
}

bool SVGLangSpace::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name().matches(XMLNames::langAttr)) {
        setXmllang(attr->value());
        return true;
    } else if (attr->name().matches(XMLNames::spaceAttr)) {
        setXmlspace(attr->value());
        return true;
    }

    return false;
}

bool SVGLangSpace::isKnownAttribute(const QualifiedName& attrName)
{
    return (attrName.matches(XMLNames::langAttr) ||
            attrName.matches(XMLNames::spaceAttr));
}

}

#endif // ENABLE(SVG)
