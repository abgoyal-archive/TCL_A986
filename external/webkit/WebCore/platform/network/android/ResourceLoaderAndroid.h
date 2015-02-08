

#ifndef ResourceLoaderAndroid_h
#define ResourceLoaderAndroid_h

#include <ResourceRequest.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class FrameLoaderClient;
class ResourceHandle;

class ResourceLoaderAndroid : public RefCounted<ResourceLoaderAndroid> {
public:
    static PassRefPtr<ResourceLoaderAndroid> start(ResourceHandle*, const ResourceRequest&, FrameLoaderClient*, bool isMainResource, bool isSync);
    virtual ~ResourceLoaderAndroid() { }

    virtual void cancel() = 0;
    virtual void downloadFile() = 0;
    // ANDROID TODO: This needs to be upstreamed.
    virtual void pauseLoad(bool) = 0;
    // END ANDROID TODO

    // Call to java to find out if this URL is in the cache
    static bool willLoadFromCache(const WebCore::KURL&, int64_t identifier);
protected:
    ResourceLoaderAndroid() { }
};

}
#endif // ResourceLoaderAndroid
