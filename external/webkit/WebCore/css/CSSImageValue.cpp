

#include "config.h"
#include "CSSImageValue.h"

#include "CSSValueKeywords.h"
#include "Cache.h"
#include "CachedImage.h"
#include "DocLoader.h"
#include "StyleCachedImage.h"

namespace WebCore {

CSSImageValue::CSSImageValue(const String& url)
    : CSSPrimitiveValue(url, CSS_URI)
    , m_accessedImage(false)
{
}

CSSImageValue::CSSImageValue()
    : CSSPrimitiveValue(CSSValueNone)
    , m_accessedImage(true)
{
}

CSSImageValue::~CSSImageValue()
{
    if (m_image)
        m_image->cachedImage()->removeClient(this);
}

StyleCachedImage* CSSImageValue::cachedImage(DocLoader* loader)
{
    return cachedImage(loader, getStringValue());
}

StyleCachedImage* CSSImageValue::cachedImage(DocLoader* loader, const String& url)
{
    if (!m_accessedImage) {
        m_accessedImage = true;

        CachedImage* cachedImage = 0;
        if (loader)
            cachedImage = loader->requestImage(url);
        else {
            // FIXME: Should find a way to make these images sit in their own memory partition, since they are user agent images.
            cachedImage = static_cast<CachedImage*>(cache()->requestResource(0, CachedResource::ImageResource, KURL(ParsedURLString, url), String()));
        }

        if (cachedImage) {
            cachedImage->addClient(this);
            m_image = StyleCachedImage::create(cachedImage);
        }
    }
    
    return m_image.get();
}

String CSSImageValue::cachedImageURL()
{
    if (!m_image)
        return String();
    return m_image->cachedImage()->url();
}

void CSSImageValue::clearCachedImage()
{
    if (m_image)
        m_image->cachedImage()->removeClient(this);
    m_image = 0;
    m_accessedImage = false;
}

} // namespace WebCore
