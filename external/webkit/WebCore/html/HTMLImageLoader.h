

#ifndef HTMLImageLoader_h
#define HTMLImageLoader_h

#include "ImageLoader.h"

namespace WebCore {

class HTMLImageLoader : public ImageLoader {
public:
    HTMLImageLoader(Element*);
    virtual ~HTMLImageLoader();

    virtual void dispatchLoadEvent();
    virtual String sourceURI(const AtomicString&) const;

    virtual void notifyFinished(CachedResource*);
};

}

#endif
