

#include "config.h"
#include "FileSystem.h"

#include "CString.h"
#include "PlatformString.h"

#include <sys/stat.h>
#ifdef ANDROID_PLUGINS
#include <sys/types.h>
#include <dirent.h>
#include <fnmatch.h>
#endif
#include <libgen.h>
#include <unistd.h>

namespace WebCore {

bool fileExists(const String& path)
{
    if (path.isNull())
        return false;

    CString fsRep = fileSystemRepresentation(path);

    if (!fsRep.data() || fsRep.data()[0] == '\0')
        return false;

    struct stat fileInfo;

    // stat(...) returns 0 on successful stat'ing of the file, and non-zero in any case where the file doesn't exist or cannot be accessed
    return !stat(fsRep.data(), &fileInfo);
}

bool deleteFile(const String& path)
{
    CString fsRep = fileSystemRepresentation(path);

    if (!fsRep.data() || fsRep.data()[0] == '\0')
        return false;

    // unlink(...) returns 0 on successful deletion of the path and non-zero in any other case (including invalid permissions or non-existent file)
    return !unlink(fsRep.data());
}

bool deleteEmptyDirectory(const String& path)
{
    CString fsRep = fileSystemRepresentation(path);

    if (!fsRep.data() || fsRep.data()[0] == '\0')
        return false;

    // rmdir(...) returns 0 on successful deletion of the path and non-zero in any other case (including invalid permissions or non-existent file)
    return !rmdir(fsRep.data());
}

bool getFileSize(const String& path, long long& result)
{
    CString fsRep = fileSystemRepresentation(path);

    if (!fsRep.data() || fsRep.data()[0] == '\0')
        return false;

    struct stat fileInfo;

    if (stat(fsRep.data(), &fileInfo))
        return false;

    result = fileInfo.st_size;
    return true;
}

bool getFileModificationTime(const String& path, time_t& result)
{
    CString fsRep = fileSystemRepresentation(path);

    if (!fsRep.data() || fsRep.data()[0] == '\0')
        return false;

    struct stat fileInfo;

    if (stat(fsRep.data(), &fileInfo))
        return false;

    result = fileInfo.st_mtime;
    return true;
}

String pathByAppendingComponent(const String& path, const String& component)
{
    if (path.endsWith("/"))
        return path + component;
    else
        return path + "/" + component;
}

bool makeAllDirectories(const String& path)
{
    CString fullPath = fileSystemRepresentation(path);
    if (!access(fullPath.data(), F_OK))
        return true;

    char* p = fullPath.mutableData() + 1;
    int length = fullPath.length();

    if(p[length - 1] == '/')
        p[length - 1] = '\0';
    for (; *p; ++p)
        if (*p == '/') {
            *p = '\0';
            if (access(fullPath.data(), F_OK))
                if (mkdir(fullPath.data(), S_IRWXU))
                    return false;
            *p = '/';
        }
    if (access(fullPath.data(), F_OK))
        if (mkdir(fullPath.data(), S_IRWXU))
            return false;

    return true;
}

String pathGetFileName(const String& path)
{
    return path.substring(path.reverseFind('/') + 1);
}

String directoryName(const String& path)
{
    CString fsRep = fileSystemRepresentation(path);

    if (!fsRep.data() || fsRep.data()[0] == '\0')
        return String();

    return dirname(fsRep.mutableData());
}

// OK to not implement listDirectory at the moment, because it's only used for plug-ins, and
// all platforms that use the shared plug-in implementation have implementations. We'd need
// to implement it if we wanted to use PluginDatabase.cpp on the Mac. Better to not implement
// at all and get a link error in case this arises, rather than having a stub here, because
// with a stub you learn about the problem at runtime instead of link time.

} // namespace WebCore
