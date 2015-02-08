

#ifndef CSSImageValue_h
#define CSSImageValue_h

#include "CSSPrimitiveValue.h"
#include "CachedResourceClient.h"
#include <wtf/RefPtr.h>

namespace WebCore {

class DocLoader;
class StyleCachedImage;

class CSSImageValue : public CSSPrimitiveValue, private CachedResourceClient {
public:
    static PassRefPtr<CSSImageValue> create() { return adoptRef(new CSSImageValue); }
    static PassRefPtr<CSSImageValue> create(const String& url) { return adoptRef(new CSSImageValue(url)); }
    virtual ~CSSImageValue();

    virtual StyleCachedImage* cachedImage(DocLoader*);
    
    virtual bool isImageValue() const { return true; }

protected:
    CSSImageValue(const String& url);

    StyleCachedImage* cachedImage(DocLoader*, const String& url);
    String cachedImageURL();
    void clearCachedImage();

private:
    CSSImageValue();

    RefPtr<StyleCachedImage> m_image;
    bool m_accessedImage;
};

} // namespace WebCore

#endif // CSSImageValue_h
