

#ifndef MediaPlayerPrivateChromium_h
#define MediaPlayerPrivateChromium_h

#if ENABLE(VIDEO)

#include "MediaPlayerPrivate.h"

namespace WebCore {

class MediaPlayerPrivate {
public:
    static void registerMediaEngine(MediaEngineRegistrar);
};

} // namespace WebCore

#endif

#endif // MediaPlayerPrivateChromium_h
