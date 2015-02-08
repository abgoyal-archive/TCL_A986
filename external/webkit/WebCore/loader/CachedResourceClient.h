

#ifndef CachedResourceClient_h
#define CachedResourceClient_h

#include <wtf/FastAllocBase.h>

#if ENABLE(XBL)
namespace XBL {
    class XBLDocument;
}
#endif

namespace WebCore {

    class CachedCSSStyleSheet;
    class CachedFont;
    class CachedResource;
    class CachedImage;
    class String;
    class Image;
    class IntRect;
    class KURL;

    /**
     * @internal
     *
     * a client who wants to load stylesheets, images or scripts from the web has to
     * inherit from this class and overload one of the 3 functions
     *
     */
    class CachedResourceClient : public FastAllocBase
    {
    public:
        virtual ~CachedResourceClient() { }

        // Called whenever a frame of an image changes, either because we got more data from the network or
        // because we are animating. If not null, the IntRect is the changed rect of the image.
        virtual void imageChanged(CachedImage*, const IntRect* = 0) { };
        
        // Called to find out if this client wants to actually display the image.  Used to tell when we
        // can halt animation.  Content nodes that hold image refs for example would not render the image,
        // but RenderImages would (assuming they have visibility: visible and their render tree isn't hidden
        // e.g., in the b/f cache or in a background tab).
        virtual bool willRenderImage(CachedImage*) { return false; }

        virtual void setCSSStyleSheet(const String& /* href */, const KURL& /* baseURL */, const String& /* charset */, const CachedCSSStyleSheet*) { }
        virtual void setXSLStyleSheet(const String& /* href */, const KURL& /* baseURL */, const String& /* sheet */) { }

        virtual void fontLoaded(CachedFont*) {};

#if ENABLE(XBL)
        virtual void setXBLDocument(const String& /*URL*/, XBL::XBLDocument*) { }
#endif

        virtual void notifyFinished(CachedResource*) { }
    };

}

#endif
