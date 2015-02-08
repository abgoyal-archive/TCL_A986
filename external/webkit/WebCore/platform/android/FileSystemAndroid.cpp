

#include "config.h"
#include "FileSystem.h"

#include "CString.h"
#include "StringBuilder.h"
#include "cutils/log.h"
#include <dirent.h>
#include <dlfcn.h>
#include <errno.h>
#include <fnmatch.h>
#include <sys/stat.h>

namespace WebCore {

// Global static used to store the base to the plugin path.
// This is set in WebSettings.cpp
String sPluginPath;

CString fileSystemRepresentation(const String& path)
{
    return path.utf8();
}

CString openTemporaryFile(const char* prefix, PlatformFileHandle& handle)
{
    int number = rand() % 10000 + 1;
    CString filename;
    do {
        StringBuilder builder;
        builder.append(sPluginPath);
        builder.append('/');
        builder.append(prefix);
        builder.append(String::number(number));
        filename = builder.toString().utf8();
        const char* fstr = filename.data();
        handle = open(filename.data(), O_WRONLY | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
        number++;
        
        if (handle != -1)
            return filename;
    } while (errno == EEXIST);
    
    return CString();
}

bool unloadModule(PlatformModule module)
{
    return !dlclose(module);
}

void closeFile(PlatformFileHandle& handle)
{
    if (isHandleValid(handle)) {
        close(handle);
        handle = invalidPlatformFileHandle;
    }
}

int writeToFile(PlatformFileHandle handle, const char* data, int length)
{
    int totalBytesWritten = 0;
    while (totalBytesWritten < length) {
        int bytesWritten = write(handle, data, (size_t)(length - totalBytesWritten));
        if (bytesWritten < 0 && errno != EINTR)
            return -1;
        if (bytesWritten > 0)
            totalBytesWritten += bytesWritten;
    }

    return totalBytesWritten;
}

String homeDirectoryPath() 
{
    return sPluginPath;
}

Vector<String> listDirectory(const String& path, const String& filter)
{
    Vector<String> entries;
    CString cpath = path.utf8();
    CString cfilter = filter.utf8();
    DIR* dir = opendir(cpath.data());
    if (dir) {
        struct dirent* dp;
        while (dp = readdir(dir)) {
            const char* name = dp->d_name;
            if (!strcmp(name, ".") || !strcmp(name, ".."))
                continue;
            if (fnmatch(cfilter.data(), name, 0))
                continue;
            char filePath[1024];
            if ((int) (sizeof(filePath) - 1) < snprintf(filePath,
                    sizeof(filePath), "%s/%s", cpath.data(), name)) {
                continue; // buffer overflow
            }
            entries.append(filePath);
        }
        closedir(dir);
    }
    return entries;
}

} // namespace WebCore
