

#ifndef HTMLVideoElement_h
#define HTMLVideoElement_h

#if ENABLE(VIDEO)

#include "HTMLMediaElement.h"
#include <wtf/OwnPtr.h>

namespace WebCore {

class HTMLImageLoader;

class HTMLVideoElement : public HTMLMediaElement {
public:
    HTMLVideoElement(const QualifiedName&, Document*);

    virtual int tagPriority() const { return 5; }
    virtual bool rendererIsNeeded(RenderStyle*);
#if !ENABLE(PLUGIN_PROXY_FOR_VIDEO)
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);
#endif
    virtual void attach();
    virtual void detach();
    virtual void parseMappedAttribute(MappedAttribute* attr);
    virtual bool isVideo() const { return true; }
    virtual bool hasVideo() const { return player() && player()->hasVideo(); }
    virtual bool supportsFullscreen() const;
    virtual bool isURLAttribute(Attribute*) const;
    virtual const QualifiedName& imageSourceAttributeName() const;

    unsigned width() const;
    void setWidth(unsigned);
    unsigned height() const;
    void setHeight(unsigned);
    
    unsigned videoWidth() const;
    unsigned videoHeight() const;
    
    const KURL& poster() const { return m_posterURL; }
    void setPoster(const String&);

// fullscreen
    void webkitEnterFullScreen(bool isUserGesture, ExceptionCode&);
    void webkitExitFullScreen();
    bool webkitSupportsFullscreen();
    bool webkitDisplayingFullscreen();

    void updatePosterImage();
    bool shouldDisplayPosterImage() const { return m_shouldDisplayPosterImage; }

    void paint(GraphicsContext*, const IntRect&);
    // Used by canvas to gain raw pixel access
    void paintCurrentFrameInContext(GraphicsContext*, const IntRect&);

private:
    virtual bool hasAvailableVideoFrame() const;

    OwnPtr<HTMLImageLoader> m_imageLoader;
    KURL m_posterURL;
    bool m_shouldDisplayPosterImage;
};

} //namespace

#endif
#endif
