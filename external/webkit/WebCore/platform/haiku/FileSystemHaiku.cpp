

#include "config.h"
#include "FileSystem.h"

#include "CString.h"
#include "PlatformString.h"

#include "NotImplemented.h"


namespace WebCore {

CString fileSystemRepresentation(const String& string)
{
    return string.utf8();
}

String homeDirectoryPath()
{
    notImplemented();
    return String();
}

CString openTemporaryFile(const char* prefix, PlatformFileHandle& handle)
{
    notImplemented();
    handle = invalidPlatformFileHandle;
    return CString();
}

void closeFile(PlatformFileHandle&)
{
    notImplemented();
}

int writeToFile(PlatformFileHandle, const char* data, int length)
{
    notImplemented();
    return 0;
}

bool unloadModule(PlatformModule)
{
    notImplemented();
    return false;
}

Vector<String> listDirectory(const String& path, const String& filter)
{
    Vector<String> entries;
    notImplemented();
    return entries;
}

} // namespace WebCore

