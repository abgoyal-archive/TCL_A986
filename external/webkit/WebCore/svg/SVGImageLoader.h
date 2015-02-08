

#ifndef SVGImageLoader_h
#define SVGImageLoader_h

#if ENABLE(SVG)
#include "ImageLoader.h"

namespace WebCore {

    class SVGImageElement;

    class SVGImageLoader : public ImageLoader {
    public:
        SVGImageLoader(SVGImageElement*);
        virtual ~SVGImageLoader();

        virtual void dispatchLoadEvent();
        virtual String sourceURI(const AtomicString&) const;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGImageLoader_h
