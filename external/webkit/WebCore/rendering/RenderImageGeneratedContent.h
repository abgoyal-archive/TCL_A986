

#ifndef RenderImageGeneratedContent_h
#define RenderImageGeneratedContent_h

#include "RenderImage.h"
#include "StyleImage.h"
#include <wtf/RefPtr.h>

namespace WebCore {

class StyleImage;

class RenderImageGeneratedContent : public RenderImage {
public:
    RenderImageGeneratedContent(Node*);
    virtual ~RenderImageGeneratedContent();
    
    void setStyleImage(StyleImage*);
    
    virtual bool hasImage() const { return true; }
    
protected:
    virtual Image* image(int w = 0, int h = 0) { return m_styleImage->image(this, IntSize(w, h)); }
    virtual bool errorOccurred() const { return m_styleImage->errorOccurred(); }
    virtual bool usesImageContainerSize() const { return m_styleImage->usesImageContainerSize(); }
    virtual void setImageContainerSize(const IntSize& size) const { m_styleImage->setImageContainerSize(size); }
    virtual bool imageHasRelativeWidth() const { return m_styleImage->imageHasRelativeWidth(); }
    virtual bool imageHasRelativeHeight() const { return m_styleImage->imageHasRelativeHeight(); }
    virtual IntSize imageSize(float multiplier) const { return m_styleImage->imageSize(this, multiplier); }
    
    // |m_styleImage| can be 0 if we get a callback for a background image from RenderObject::setStyle.
    virtual WrappedImagePtr imagePtr() const { return m_styleImage ? m_styleImage->data() : 0; }

private:
    RefPtr<StyleImage> m_styleImage;
};

} // namespace WebCore

#endif // RenderImageGeneratedContent_h
