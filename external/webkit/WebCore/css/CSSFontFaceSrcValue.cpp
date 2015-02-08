

#include "config.h"
#include "CSSFontFaceSrcValue.h"
#include "CSSStyleSheet.h"
#include "Node.h"

namespace WebCore {

#if ENABLE(SVG_FONTS)
bool CSSFontFaceSrcValue::isSVGFontFaceSrc() const
{
    return equalIgnoringCase(m_format, "svg");
}
#endif

bool CSSFontFaceSrcValue::isSupportedFormat() const
{
    // Normally we would just check the format, but in order to avoid conflicts with the old WinIE style of font-face,
    // we will also check to see if the URL ends with .eot.  If so, we'll go ahead and assume that we shouldn't load it.
    if (m_format.isEmpty()) {
        // Check for .eot.
        if (m_resource.endsWith("eot", false))
            return false;
        return true;
    }

    return equalIgnoringCase(m_format, "truetype") || equalIgnoringCase(m_format, "opentype")
#if ENABLE(SVG_FONTS)
           || isSVGFontFaceSrc()
#endif
           ;
}

String CSSFontFaceSrcValue::cssText() const
{
    String result;
    if (isLocal())
        result += "local(";
    else
        result += "url(";
    result += m_resource;
    result += ")";
    if (!m_format.isEmpty())
        result += " format(" + m_format + ")";
    return result;
}

void CSSFontFaceSrcValue::addSubresourceStyleURLs(ListHashSet<KURL>& urls, const CSSStyleSheet* styleSheet)
{
    if (!isLocal())
        addSubresourceURL(urls, styleSheet->completeURL(m_resource));
}

}

