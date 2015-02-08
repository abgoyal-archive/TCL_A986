

#include "config.h"
#include "FileSystem.h"

#include "ChromiumBridge.h"
#include "NotImplemented.h"
#include "PlatformString.h"

namespace WebCore {

bool deleteFile(const String& path)
{
    return ChromiumBridge::deleteFile(path);
}

bool deleteEmptyDirectory(const String& path)
{
    return ChromiumBridge::deleteEmptyDirectory(path);
}

bool getFileSize(const String& path, long long& result)
{
    return ChromiumBridge::getFileSize(path, result);
}

bool getFileModificationTime(const String& path, time_t& result)
{
    return ChromiumBridge::getFileModificationTime(path, result);
}

String directoryName(const String& path)
{
    return ChromiumBridge::directoryName(path);
}

String pathByAppendingComponent(const String& path, const String& component)
{
    return ChromiumBridge::pathByAppendingComponent(path, component);
}

bool makeAllDirectories(const String& path)
{
    return ChromiumBridge::makeAllDirectories(path);
}

bool fileExists(const String& path)
{
    return ChromiumBridge::fileExists(path);
}

} // namespace WebCore
