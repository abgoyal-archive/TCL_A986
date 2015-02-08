

#include "config.h"
#include "FileSystem.h"

namespace WebCore {

String pathGetFileName(const String& path)
{
    return path.substring(path.reverseFind('/') + 1);
}

String pathGetDisplayFileName(const String& path)
{
    return pathGetFileName(path);
}

} // namespace WebCore
