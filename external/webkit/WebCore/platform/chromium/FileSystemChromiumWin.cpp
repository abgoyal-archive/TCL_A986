

#include "config.h"
#include "FileSystem.h"

#include <windows.h>
#include <shlwapi.h>

namespace WebCore {

String pathGetFileName(const String& path)
{
    return String(PathFindFileName(String(path).charactersWithNullTermination()));
}

String pathGetDisplayFileName(const String& path)
{
    return pathGetFileName(path);
}

} // namespace WebCore
