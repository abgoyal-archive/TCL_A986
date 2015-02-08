

#include "config.h"

#include "KURL.h"
#include "NotImplemented.h"
#include "SharedBuffer.h"

namespace WebCore {

String KURL::fileSystemPath() const
{
    notImplemented();
    return String();
}

PassRefPtr<SharedBuffer> SharedBuffer::createWithContentsOfFile(const String&)
{
    notImplemented();
    return 0;
}

} // namespace WebCore
