

#include "config.h"
#include "SharedBuffer.h"

#include <File.h>
#include <String.h>

namespace WebCore {

PassRefPtr<SharedBuffer> SharedBuffer::createWithContentsOfFile(const String& fileName)
{
    if (fileName.isEmpty())
        return 0;

    BFile file(BString(fileName).String(), B_READ_ONLY);
    if (file.InitCheck() != B_OK)
        return 0;

    RefPtr<SharedBuffer> result = SharedBuffer::create();

    off_t size;
    file.GetSize(&size);
    result->m_buffer.resize(size);
    if (result->m_buffer.size() != size)
        return 0;
    result->m_size = size;

    file.Read(result->m_buffer.data(), result->m_buffer.size());
    return result.release();
}

} // namespace WebCore
