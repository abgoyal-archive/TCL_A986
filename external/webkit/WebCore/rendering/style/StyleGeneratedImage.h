

#ifndef StyleGeneratedImage_h
#define StyleGeneratedImage_h

#include "StyleImage.h"

namespace WebCore {

class CSSValue;
class CSSImageGeneratorValue;

class StyleGeneratedImage : public StyleImage {
public:
    static PassRefPtr<StyleGeneratedImage> create(CSSImageGeneratorValue* val, bool fixedSize)
    {
        return adoptRef(new StyleGeneratedImage(val, fixedSize));
    }

    virtual WrappedImagePtr data() const { return m_generator; }

    virtual bool isGeneratedImage() const { return true; }
    
    virtual PassRefPtr<CSSValue> cssValue();

    virtual IntSize imageSize(const RenderObject*, float multiplier) const;
    virtual bool imageHasRelativeWidth() const { return !m_fixedSize; }
    virtual bool imageHasRelativeHeight() const { return !m_fixedSize; }
    virtual bool usesImageContainerSize() const { return !m_fixedSize; }
    virtual void setImageContainerSize(const IntSize&);
    virtual void addClient(RenderObject*);
    virtual void removeClient(RenderObject*);
    virtual Image* image(RenderObject*, const IntSize&) const;
    
private:
    StyleGeneratedImage(CSSImageGeneratorValue* val, bool fixedSize)
        : m_generator(val)
        , m_fixedSize(fixedSize)
    {
    }
    
    CSSImageGeneratorValue* m_generator; // The generator holds a reference to us.
    IntSize m_containerSize;
    bool m_fixedSize;
};

}
#endif
