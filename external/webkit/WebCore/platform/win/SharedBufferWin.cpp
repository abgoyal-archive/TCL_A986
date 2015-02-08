

#include "config.h"
#include "SharedBuffer.h"

namespace WebCore {

PassRefPtr<SharedBuffer> SharedBuffer::createWithContentsOfFile(const String& filePath)
{
    if (filePath.isEmpty())
        return 0;

    String nullifiedPath = filePath;
    FILE* fileDescriptor = 0;
    if (_wfopen_s(&fileDescriptor, nullifiedPath.charactersWithNullTermination(), TEXT("rb")) || !fileDescriptor) {
        LOG_ERROR("Failed to open file %s to create shared buffer, errno(%i)", filePath.ascii().data(), errno);
        return 0;
    }

    RefPtr<SharedBuffer> result;

    // Stat the file to get its size
    struct _stat64 fileStat;
    if (_fstat64(_fileno(fileDescriptor), &fileStat))
        goto exit;

    result = SharedBuffer::create();
    result->m_buffer.resize(fileStat.st_size);
    if (result->m_buffer.size() != fileStat.st_size) {
        result = 0;
        goto exit;
    }

    result->m_size = result->m_buffer.size();

    if (fread(result->m_buffer.data(), 1, fileStat.st_size, fileDescriptor) != fileStat.st_size)
        LOG_ERROR("Failed to fully read contents of file %s - errno(%i)", filePath.ascii().data(), errno);

exit:
    fclose(fileDescriptor);
    return result.release();
}

}; // namespace WebCore
