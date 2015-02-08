

#ifndef WebURLResponsePrivate_h
#define WebURLResponsePrivate_h

namespace WebCore { class ResourceResponse; }

namespace WebKit {

class WebURLResponsePrivate {
public:
    WebURLResponsePrivate() : m_resourceResponse(0) { }

    // Called by WebURLResponse when it no longer needs this object.
    virtual void dispose() = 0;

    WebCore::ResourceResponse* m_resourceResponse;
};

} // namespace WebKit

#endif
