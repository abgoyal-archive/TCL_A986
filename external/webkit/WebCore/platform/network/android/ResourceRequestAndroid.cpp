

#include "config.h"
#include "ResourceRequest.h"

namespace WebCore {

unsigned initializeMaximumHTTPConnectionCountPerHost()
{
    // This is used by the loader to control the number of parallel load
    // requests. Our java framework has 4 threads that can each pipeline up to
    // 5 requests. Use 20 as a maximum number.
    return 20;
}

} // namespace WebCore
