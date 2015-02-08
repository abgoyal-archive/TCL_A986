

#ifndef RenderImage_h
#define RenderImage_h

#include "RenderReplaced.h"

namespace WebCore {

class HTMLMapElement;

class RenderImage : public RenderReplaced {
public:
    RenderImage(Node*);
    virtual ~RenderImage();

    bool setImageSizeForAltText(CachedImage* newImage = 0);

    void updateAltText();

    void setCachedImage(CachedImage*);
    CachedImage* cachedImage() const { return m_cachedImage.get(); }

    HTMLMapElement* imageMap() const;

    void resetAnimation();

    virtual bool hasImage() const { return m_cachedImage; }

    void highQualityRepaintTimerFired(Timer<RenderImage>*);

protected:
    virtual Image* image(int /* width */ = 0, int /* height */ = 0) { return m_cachedImage ? m_cachedImage->image() : nullImage(); }
    virtual bool errorOccurred() const { return m_cachedImage && m_cachedImage->errorOccurred(); }

    virtual void imageChanged(WrappedImagePtr, const IntRect* = 0);

    virtual void paintIntoRect(GraphicsContext*, const IntRect&);
    void paintFocusRings(PaintInfo&, const RenderStyle*);
    virtual void paint(PaintInfo&, int tx, int ty);

    bool isWidthSpecified() const;
    bool isHeightSpecified() const;

    virtual void intrinsicSizeChanged() { imageChanged(imagePtr()); }

private:
    virtual const char* renderName() const { return "RenderImage"; }

    virtual bool isImage() const { return true; }
    virtual bool isRenderImage() const { return true; }
    
    virtual void paintReplaced(PaintInfo&, int tx, int ty);

    virtual int minimumReplacedHeight() const;

    virtual void notifyFinished(CachedResource*);
    virtual bool nodeAtPoint(const HitTestRequest&, HitTestResult&, int x, int y, int tx, int ty, HitTestAction);

    virtual int calcReplacedWidth(bool includeMaxWidth = true) const;
    virtual int calcReplacedHeight() const;

    virtual void calcPrefWidths();

    virtual bool usesImageContainerSize() const { return m_cachedImage ? m_cachedImage->usesImageContainerSize() : false; }
    virtual void setImageContainerSize(const IntSize& size) const { if (m_cachedImage) m_cachedImage->setImageContainerSize(size); }
    virtual bool imageHasRelativeWidth() const { return m_cachedImage ? m_cachedImage->imageHasRelativeWidth() : false; }
    virtual bool imageHasRelativeHeight() const { return m_cachedImage ? m_cachedImage->imageHasRelativeHeight() : false; }
    virtual IntSize imageSize(float multiplier) const { return m_cachedImage ? m_cachedImage->imageSize(multiplier) : IntSize(); }
    virtual WrappedImagePtr imagePtr() const { return m_cachedImage.get(); }

    int calcAspectRatioWidth() const;
    int calcAspectRatioHeight() const;

protected:
    // The image we are rendering.
    CachedResourceHandle<CachedImage> m_cachedImage;

private:
    // Text to display as long as the image isn't available.
    String m_altText;

    static Image* nullImage();
    
    friend class RenderImageScaleObserver;
};

inline RenderImage* toRenderImage(RenderObject* object)
{ 
    ASSERT(!object || object->isRenderImage());
    return static_cast<RenderImage*>(object);
}

inline const RenderImage* toRenderImage(const RenderObject* object)
{ 
    ASSERT(!object || object->isRenderImage());
    return static_cast<const RenderImage*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderImage(const RenderImage*);

} // namespace WebCore

#endif // RenderImage_h
