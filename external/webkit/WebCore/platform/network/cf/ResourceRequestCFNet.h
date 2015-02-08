

#ifndef ResourceRequestCFNet_h
#define ResourceRequestCFNet_h

typedef const struct _CFURLRequest* CFURLRequestRef;

namespace WebCore {

    class ResourceRequest;

    void getResourceRequest(ResourceRequest&, CFURLRequestRef);
    CFURLRequestRef cfURLRequest(const ResourceRequest&);
}

#endif // ResourceRequestCFNet_h
