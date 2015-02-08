

#include "config.h"
#include "ResourceRequest.h"

namespace WebCore {

// This is used by the loader to control the number of issued parallel load requests. 
unsigned initializeMaximumHTTPConnectionCountPerHost()
{
    return 6;
}

} // namespace WebCore
