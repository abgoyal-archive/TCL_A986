

#include "config.h"
#include "Blob.h"

#include "FileSystem.h"

namespace WebCore {

Blob::Blob(const String& path)
    : m_path(path)
{
}

unsigned long long Blob::size() const
{
    // FIXME: JavaScript cannot represent sizes as large as unsigned long long, we need to
    // come up with an exception to throw if file size is not represetable.
    long long size;
    if (!getFileSize(m_path, size))
        return 0;
    return static_cast<unsigned long long>(size);
}

} // namespace WebCore
