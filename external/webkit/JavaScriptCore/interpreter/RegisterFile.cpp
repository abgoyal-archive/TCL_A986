

#include "config.h"
#include "RegisterFile.h"

namespace JSC {

RegisterFile::~RegisterFile()
{
#if HAVE(MMAP)
    munmap(m_buffer, ((m_max - m_start) + m_maxGlobals) * sizeof(Register));
#elif HAVE(VIRTUALALLOC)
#if OS(WINCE)
    VirtualFree(m_buffer, DWORD(m_commitEnd) - DWORD(m_buffer), MEM_DECOMMIT);
#endif
    VirtualFree(m_buffer, 0, MEM_RELEASE);
#else
    fastFree(m_buffer);
#endif
}

void RegisterFile::releaseExcessCapacity()
{
#if HAVE(MMAP) && HAVE(MADV_FREE) && !HAVE(VIRTUALALLOC)
    while (madvise(m_start, (m_max - m_start) * sizeof(Register), MADV_FREE) == -1 && errno == EAGAIN) { }
#elif HAVE(VIRTUALALLOC)
    VirtualFree(m_start, (m_max - m_start) * sizeof(Register), MEM_DECOMMIT);
    m_commitEnd = m_start;
#endif
    m_maxUsed = m_start;
}

} // namespace JSC
