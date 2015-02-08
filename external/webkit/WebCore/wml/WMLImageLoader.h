

#ifndef WMLImageLoader_h
#define WMLImageLoader_h

#if ENABLE(WML)
#include "ImageLoader.h"

namespace WebCore {

class WMLImageElement;

class WMLImageLoader : public ImageLoader {
public:
    WMLImageLoader(WMLImageElement*);
    virtual ~WMLImageLoader();

    virtual void dispatchLoadEvent();
    virtual String sourceURI(const AtomicString&) const;

    virtual void notifyFinished(CachedResource*);
};

}

#endif
#endif
