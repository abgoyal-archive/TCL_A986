

#include "config.h"
#include "ResourceLoader.h"

#include "FrameLoader.h"
#include "FrameLoaderClient.h"

namespace WebCore {

bool ResourceLoader::shouldCacheResponse(ResourceHandle*, CFCachedURLResponseRef cachedResponse)
{
    if (!m_sendResourceLoadCallbacks)
        return 0;

    CFURLResponseRef response = CFCachedURLResponseGetWrappedResponse(cachedResponse);
    CFDataRef data = CFCachedURLResponseGetReceiverData(cachedResponse);
    return frameLoader()->client()->shouldCacheResponse(documentLoader(), identifier(), response, CFDataGetBytePtr(data), CFDataGetLength(data));
}

} // namespace WebCore
