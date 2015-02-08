

#ifndef CachedImage_h
#define CachedImage_h

#include "CachedResource.h"
#include "ImageObserver.h"
#include "Image.h"
#include "IntRect.h"
#include "Timer.h"
#include <wtf/Vector.h>

namespace WebCore {

class DocLoader;
class Cache;

class CachedImage : public CachedResource, public ImageObserver {
    friend class Cache;

public:
    CachedImage(const String& url);
    CachedImage(Image*);
    virtual ~CachedImage();
    
    virtual void load(DocLoader* docLoader);

    Image* image() const;

    bool canRender(float multiplier) const { return !errorOccurred() && !imageSize(multiplier).isEmpty(); }

    // These are only used for SVGImage right now
    void setImageContainerSize(const IntSize&);
    bool usesImageContainerSize() const;
    bool imageHasRelativeWidth() const;
    bool imageHasRelativeHeight() const;
    
    // Both of these methods take a zoom multiplier that can be used to increase the natural size of the image by the
    // zoom.
    IntSize imageSize(float multiplier) const;  // returns the size of the complete image.
    IntRect imageRect(float multiplier) const;  // The size of the currently decoded portion of the image.

    virtual void didAddClient(CachedResourceClient*);
    
    virtual void allClientsRemoved();
    virtual void destroyDecodedData();

    virtual void data(PassRefPtr<SharedBuffer> data, bool allDataReceived);
    virtual void error();
    
    virtual void httpStatusCodeError() { m_httpStatusCodeErrorOccurred = true; }
    bool httpStatusCodeErrorOccurred() const { return m_httpStatusCodeErrorOccurred; }
    
    virtual bool schedule() const { return true; }

    void checkNotify();
    
    virtual bool isImage() const { return true; }

    void clear();
    
    bool stillNeedsLoad() const { return !m_errorOccurred && m_status == Unknown && m_loading == false; }
    void load();

    // ImageObserver
    virtual void decodedSizeChanged(const Image* image, int delta);
    virtual void didDraw(const Image*);

    virtual bool shouldPauseAnimation(const Image*);
    virtual void animationAdvanced(const Image*);
    virtual void changedInRect(const Image*, const IntRect&);

private:
    void createImage();
    size_t maximumDecodedImageSize();
    // If not null, changeRect is the changed part of the image.
    void notifyObservers(const IntRect* changeRect = 0);
    void decodedDataDeletionTimerFired(Timer<CachedImage>*);

    RefPtr<Image> m_image;
    Timer<CachedImage> m_decodedDataDeletionTimer;
    bool m_httpStatusCodeErrorOccurred;
};

}

#endif
