

#include "config.h"
#include "File.h"

#include "FileSystem.h"

namespace WebCore {

File::File(const String& path)
    : Blob(path)
    , m_name(pathGetFileName(path))
{
}

} // namespace WebCore
