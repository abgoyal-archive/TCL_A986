

#include "config.h"

#include "CrossThreadCopier.h"

#include "KURL.h"
#include "PlatformString.h"
#include "ResourceError.h"
#include "ResourceRequest.h"
#include "ResourceResponse.h"

namespace WebCore {

CrossThreadCopierBase<false, false, KURL>::Type CrossThreadCopierBase<false, false, KURL>::copy(const KURL& url)
{
    return url.copy();
}

CrossThreadCopierBase<false, false, String>::Type CrossThreadCopierBase<false, false, String>::copy(const String& str)
{
    return str.crossThreadString();
}

CrossThreadCopierBase<false, false, ResourceError>::Type CrossThreadCopierBase<false, false, ResourceError>::copy(const ResourceError& error)
{
    return error.copy();
}

CrossThreadCopierBase<false, false, ResourceRequest>::Type CrossThreadCopierBase<false, false, ResourceRequest>::copy(const ResourceRequest& request)
{
    return request.copyData();
}

CrossThreadCopierBase<false, false, ResourceResponse>::Type CrossThreadCopierBase<false, false, ResourceResponse>::copy(const ResourceResponse& response)
{
    return response.copyData();
}

} // namespace WebCore
