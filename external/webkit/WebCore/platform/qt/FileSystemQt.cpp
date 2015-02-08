

#include "config.h"
#include "FileSystem.h"

#include "CString.h"
#include "PlatformString.h"

#include <QDateTime>
#include <QFile>
#include <QTemporaryFile>
#include <QFileInfo>
#include <QDateTime>
#include <QDir>

namespace WebCore {

bool fileExists(const String& path)
{
    return QFile::exists(path);
}


bool deleteFile(const String& path)
{
    return QFile::remove(path);
}

bool deleteEmptyDirectory(const String& path)
{
    return QDir::root().rmdir(path);
}

bool getFileSize(const String& path, long long& result)
{
    QFileInfo info(path);
    result = info.size();
    return info.exists();
}

bool getFileModificationTime(const String& path, time_t& result)
{
    QFileInfo info(path);
    result = info.lastModified().toTime_t();
    return info.exists();
}

bool makeAllDirectories(const String& path)
{
    return QDir::root().mkpath(path);
}

String pathByAppendingComponent(const String& path, const String& component)
{
    return QDir::toNativeSeparators(QDir(path).filePath(component));
}

String homeDirectoryPath()
{
    return QDir::homePath();
}

String pathGetFileName(const String& path)
{
    return QFileInfo(path).fileName();
}

String directoryName(const String& path)
{
    return String(QFileInfo(path).absolutePath());
}

Vector<String> listDirectory(const String& path, const String& filter)
{
    Vector<String> entries;

    QStringList nameFilters;
    if (!filter.isEmpty())
        nameFilters.append(filter);
    QFileInfoList fileInfoList = QDir(path).entryInfoList(nameFilters, QDir::AllEntries | QDir::NoDotAndDotDot);
    foreach (const QFileInfo fileInfo, fileInfoList) {
        String entry = String(fileInfo.canonicalFilePath());
        entries.append(entry);
    }

    return entries;
}

CString openTemporaryFile(const char* prefix, PlatformFileHandle& handle)
{
    QTemporaryFile* tempFile = new QTemporaryFile(QLatin1String(prefix));
    tempFile->setAutoRemove(false);
    QFile* temp = tempFile;
    if (temp->open(QIODevice::ReadWrite)) {
        handle = temp;
        return String(temp->fileName()).utf8();
    }
    handle = invalidPlatformFileHandle;
    return CString();
}

void closeFile(PlatformFileHandle& handle)
{
    if (handle) {
        handle->close();
        delete handle;
    }
}

int writeToFile(PlatformFileHandle handle, const char* data, int length)
{
    if (handle && handle->exists() && handle->isWritable())
        return handle->write(data, length);

    return 0;
}

bool unloadModule(PlatformModule module)
{
#if defined(Q_WS_MAC)
    CFRelease(module);
    return true;

#elif defined(Q_OS_WIN)
    return ::FreeLibrary(module);

#else
    if (module->unload()) {
        delete module;
        return true;
    }

    return false;
#endif
}

}

// vim: ts=4 sw=4 et
