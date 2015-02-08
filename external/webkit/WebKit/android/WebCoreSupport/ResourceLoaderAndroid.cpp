

#include <config.h>

#include <ResourceLoaderAndroid.h>

#include "FrameLoaderClientAndroid.h"
#include "WebCoreFrameBridge.h"
#include "WebCoreResourceLoader.h"

using namespace android;

namespace WebCore {

PassRefPtr<ResourceLoaderAndroid> ResourceLoaderAndroid::start(
        ResourceHandle* handle, const ResourceRequest& request, FrameLoaderClient* client, bool isMainResource, bool isSync)
{
    FrameLoaderClientAndroid* clientAndroid = static_cast<FrameLoaderClientAndroid*> (client);
    return clientAndroid->webFrame()->startLoadingResource(handle, request, isMainResource, isSync);
}

bool ResourceLoaderAndroid::willLoadFromCache(const WebCore::KURL& url, int64_t identifier)
{
    return WebCoreResourceLoader::willLoadFromCache(url, identifier);
}

}
