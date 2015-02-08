

#ifndef CSSImageGeneratorValue_h
#define CSSImageGeneratorValue_h

#include "CSSValue.h"
#include "IntSizeHash.h"
#include <wtf/HashMap.h>
#include <wtf/HashCountedSet.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Image;
class StyleGeneratedImage;
class RenderObject;

class CSSImageGeneratorValue : public CSSValue {
public:
    virtual ~CSSImageGeneratorValue();

    void addClient(RenderObject*, const IntSize&);
    void removeClient(RenderObject*);
    virtual Image* image(RenderObject*, const IntSize&) = 0;

    StyleGeneratedImage* generatedImage();
    
    virtual bool isFixedSize() const { return false; }
    virtual IntSize fixedSize(const RenderObject*) { return IntSize(); }
    
protected:
    CSSImageGeneratorValue();
    
    Image* getImage(RenderObject*, const IntSize&);
    void putImage(const IntSize&, PassRefPtr<Image>);

    typedef pair<IntSize, int> SizeCountPair;
    typedef HashMap<RenderObject*, SizeCountPair> RenderObjectSizeCountMap;

    HashCountedSet<IntSize> m_sizes; // A count of how many times a given image size is in use.
    RenderObjectSizeCountMap m_clients; // A map from RenderObjects (with entry count) to image sizes.
    HashMap<IntSize, RefPtr<Image> > m_images; // A cache of Image objects by image size.
    
    RefPtr<StyleGeneratedImage> m_image;
    bool m_accessedImage;

private:
    virtual bool isImageGeneratorValue() const { return true; }
};

} // namespace WebCore

#endif // CSSImageGeneratorValue_h
