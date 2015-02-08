

#include "config.h"
#include "SharedBuffer.h"

#include <QFile>

namespace WebCore {

PassRefPtr<SharedBuffer> SharedBuffer::createWithContentsOfFile(const String& fileName)
{
    if (fileName.isEmpty())
        return 0;

    QFile file(fileName);
    if (!file.exists() || !file.open(QFile::ReadOnly))
        return 0;


    RefPtr<SharedBuffer> result = SharedBuffer::create();
    result->m_buffer.resize(file.size());
    if (result->m_buffer.size() != file.size())
        return 0;

    result->m_size = result->m_buffer.size();

    file.read(result->m_buffer.data(), result->m_buffer.size());
    return result.release();
}

}
