

#ifndef StyleCachedImage_h
#define StyleCachedImage_h

#include "CachedResourceHandle.h"
#include "StyleImage.h"

namespace WebCore {

class CachedImage;

class StyleCachedImage : public StyleImage {
public:
    static PassRefPtr<StyleCachedImage> create(CachedImage* image) { return adoptRef(new StyleCachedImage(image)); }
    virtual WrappedImagePtr data() const { return m_image.get(); }

    virtual bool isCachedImage() const { return true; }
    
    virtual PassRefPtr<CSSValue> cssValue();
    
    CachedImage* cachedImage() const { return m_image.get(); }

    virtual bool canRender(float multiplier) const;
    virtual bool isLoaded() const;
    virtual bool errorOccurred() const;
    virtual IntSize imageSize(const RenderObject*, float multiplier) const;
    virtual bool imageHasRelativeWidth() const;
    virtual bool imageHasRelativeHeight() const;
    virtual bool usesImageContainerSize() const;
    virtual void setImageContainerSize(const IntSize&);
    virtual void addClient(RenderObject*);
    virtual void removeClient(RenderObject*);
    virtual Image* image(RenderObject*, const IntSize&) const;
    
private:
    StyleCachedImage(CachedImage* image)
        : m_image(image)
    {
    }
    
    CachedResourceHandle<CachedImage> m_image;
};

}
#endif
