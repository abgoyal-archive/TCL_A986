

#include "config.h"
#include "MediaPlayerPrivateChromium.h"

#if ENABLE(VIDEO)

#include "WebMediaPlayerClientImpl.h"

namespace WebCore {

void MediaPlayerPrivate::registerMediaEngine(MediaEngineRegistrar registrar)
{
    WebKit::WebMediaPlayerClientImpl::registerSelf(registrar);
}

} // namespace WebCore

#endif
