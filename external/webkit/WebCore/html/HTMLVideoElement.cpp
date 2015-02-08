

#include "config.h"

#if ENABLE(VIDEO)
#include "HTMLVideoElement.h"

#include "Chrome.h"
#include "ChromeClient.h"
#include "CSSHelper.h"
#include "CSSPropertyNames.h"
#include "Document.h"
#include "ExceptionCode.h"
#include "HTMLImageLoader.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"
#include "Page.h"
#include "RenderImage.h"
#include "RenderVideo.h"

namespace WebCore {

using namespace HTMLNames;

HTMLVideoElement::HTMLVideoElement(const QualifiedName& tagName, Document* doc)
    : HTMLMediaElement(tagName, doc)
    , m_shouldDisplayPosterImage(false)
{
    ASSERT(hasTagName(videoTag));
}

bool HTMLVideoElement::rendererIsNeeded(RenderStyle* style) 
{
    return HTMLElement::rendererIsNeeded(style); 
}

#if !ENABLE(PLUGIN_PROXY_FOR_VIDEO)
RenderObject* HTMLVideoElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderVideo(this);
}
#endif

void HTMLVideoElement::attach()
{
    HTMLMediaElement::attach();

#if !ENABLE(PLUGIN_PROXY_FOR_VIDEO)
    updatePosterImage();
    if (m_shouldDisplayPosterImage) {
        if (!m_imageLoader)
            m_imageLoader.set(new HTMLImageLoader(this));
        m_imageLoader->updateFromElement();
        if (renderer()) {
            RenderImage* imageRenderer = toRenderImage(renderer());
            imageRenderer->setCachedImage(m_imageLoader->image()); 
        }
    }
#endif
}

void HTMLVideoElement::detach()
{
    HTMLMediaElement::detach();
    
    if (!m_shouldDisplayPosterImage)
        if (m_imageLoader)
            m_imageLoader.clear();
}

void HTMLVideoElement::parseMappedAttribute(MappedAttribute* attr)
{
    const QualifiedName& attrName = attr->name();

    if (attrName == posterAttr) {
        m_posterURL = document()->completeURL(attr->value());
        updatePosterImage();
        if (m_shouldDisplayPosterImage) {
#if !ENABLE(PLUGIN_PROXY_FOR_VIDEO)
            if (!m_imageLoader)
                m_imageLoader.set(new HTMLImageLoader(this));
            m_imageLoader->updateFromElementIgnoringPreviousError();
#else
            if (m_player)
                m_player->setPoster(poster());
#endif
        }
    } else if (attrName == widthAttr)
        addCSSLength(attr, CSSPropertyWidth, attr->value());
    else if (attrName == heightAttr)
        addCSSLength(attr, CSSPropertyHeight, attr->value());
    else
        HTMLMediaElement::parseMappedAttribute(attr);
}

bool HTMLVideoElement::supportsFullscreen() const
{
    Page* page = document() ? document()->page() : 0;
    if (!page) 
        return false;

    if (!m_player || !m_player->supportsFullscreen() || !m_player->hasVideo())
        return false;

    // Check with the platform client.
    return page->chrome()->client()->supportsFullscreenForNode(this);
}

unsigned HTMLVideoElement::videoWidth() const
{
    if (!m_player)
        return 0;
    return m_player->naturalSize().width();
}

unsigned HTMLVideoElement::videoHeight() const
{
    if (!m_player)
        return 0;
    return m_player->naturalSize().height();
}

unsigned HTMLVideoElement::width() const
{
    bool ok;
    unsigned w = getAttribute(widthAttr).string().toUInt(&ok);
    return ok ? w : 0;
}

void HTMLVideoElement::setWidth(unsigned value)
{
    setAttribute(widthAttr, String::number(value));
}
    
unsigned HTMLVideoElement::height() const
{
    bool ok;
    unsigned h = getAttribute(heightAttr).string().toUInt(&ok);
    return ok ? h : 0;
}
    
void HTMLVideoElement::setHeight(unsigned value)
{
    setAttribute(heightAttr, String::number(value));
}

void HTMLVideoElement::setPoster(const String& value)
{
    setAttribute(posterAttr, value);
}

bool HTMLVideoElement::isURLAttribute(Attribute* attr) const
{
    return attr->name() == posterAttr;
}

const QualifiedName& HTMLVideoElement::imageSourceAttributeName() const
{
    return posterAttr;
}

void HTMLVideoElement::updatePosterImage()
{
#if !ENABLE(PLUGIN_PROXY_FOR_VIDEO)
    bool oldShouldShowPosterImage = m_shouldDisplayPosterImage;
#endif

    m_shouldDisplayPosterImage = !poster().isEmpty() && !hasAvailableVideoFrame();

#if !ENABLE(PLUGIN_PROXY_FOR_VIDEO)
    if (renderer() && oldShouldShowPosterImage != m_shouldDisplayPosterImage)
        renderer()->updateFromElement();
#endif
}

void HTMLVideoElement::paint(GraphicsContext* context, const IntRect& destRect)
{
    MediaPlayer* player = HTMLMediaElement::player();
    if (!player)
        return;

    player->setVisible(true); // Make player visible or it won't draw.
    player->paint(context, destRect);
}

void HTMLVideoElement::paintCurrentFrameInContext(GraphicsContext* context, const IntRect& destRect)
{
    MediaPlayer* player = HTMLMediaElement::player();
    if (!player)
        return;
    
    player->setVisible(true); // Make player visible or it won't draw.
    player->paintCurrentFrameInContext(context, destRect);
}

bool HTMLVideoElement::hasAvailableVideoFrame() const
{
    if (!m_player)
        return false;
    
    return m_player->hasAvailableVideoFrame();
}

void HTMLVideoElement::webkitEnterFullScreen(bool isUserGesture, ExceptionCode& ec)
{
    if (m_isFullscreen)
        return;

    // Generate an exception if this isn't called in response to a user gesture, or if the 
    // element does not support fullscreen.
    if (!isUserGesture || !supportsFullscreen()) {
        ec = INVALID_STATE_ERR;
        return;
    }

    enterFullscreen();
}

void HTMLVideoElement::webkitExitFullScreen()
{
    if (m_isFullscreen)
        exitFullscreen();
}

bool HTMLVideoElement::webkitSupportsFullscreen()
{
    return supportsFullscreen();
}

bool HTMLVideoElement::webkitDisplayingFullscreen()
{
    return m_isFullscreen;
}


}
#endif
