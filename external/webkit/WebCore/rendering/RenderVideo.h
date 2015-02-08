

#ifndef RenderVideo_h
#define RenderVideo_h

#if ENABLE(VIDEO)

#include "RenderMedia.h"

namespace WebCore {
    
class HTMLMediaElement;
class HTMLVideoElement;
#if USE(ACCELERATED_COMPOSITING)
class GraphicsLayer;
#endif

class RenderVideo : public RenderMedia {
public:
    RenderVideo(HTMLVideoElement*);
    virtual ~RenderVideo();

    void videoSizeChanged();
    IntRect videoBox() const;
    
#if USE(ACCELERATED_COMPOSITING)
    bool supportsAcceleratedRendering() const;
    void acceleratedRenderingStateChanged();
    GraphicsLayer* videoGraphicsLayer() const;
#endif

private:
    virtual void updateFromElement();
    inline HTMLVideoElement* videoElement() const;

    virtual void intrinsicSizeChanged();
    virtual void imageChanged(WrappedImagePtr, const IntRect*);

    virtual const char* renderName() const { return "RenderVideo"; }

    virtual bool requiresLayer() const { return true; }
    virtual bool isVideo() const { return true; }

    virtual void paintReplaced(PaintInfo&, int tx, int ty);

    virtual void layout();

    virtual int calcReplacedWidth(bool includeMaxWidth = true) const;
    virtual int calcReplacedHeight() const;
    virtual int minimumReplacedHeight() const;
    
    int calcAspectRatioWidth() const;
    int calcAspectRatioHeight() const;

    void updatePlayer();

    IntSize m_cachedImageSize;
};

inline RenderVideo* toRenderVideo(RenderObject* object)
{
    ASSERT(!object || object->isVideo());
    return static_cast<RenderVideo*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderVideo(const RenderVideo*);

} // namespace WebCore

#endif
#endif // RenderVideo_h
