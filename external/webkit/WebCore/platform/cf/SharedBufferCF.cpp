
#include "config.h"
#include "SharedBuffer.h"

#include "PurgeableBuffer.h"

namespace WebCore {

SharedBuffer::SharedBuffer(CFDataRef cfData)
    : m_size(0)
    , m_cfData(cfData)
{
}

// Mac is a CF platform but has an even more efficient version of this method,
// so only use this version for non-Mac
#if !PLATFORM(MAC)
CFDataRef SharedBuffer::createCFData()
{
    if (m_cfData) {
        CFRetain(m_cfData.get());
        return m_cfData.get();
    }

    return CFDataCreate(0, reinterpret_cast<const UInt8*>(m_buffer.data()), m_buffer.size());
}
#endif

PassRefPtr<SharedBuffer> SharedBuffer::wrapCFData(CFDataRef data)
{
    return adoptRef(new SharedBuffer(data));
}

bool SharedBuffer::hasPlatformData() const
{
    return m_cfData;
}

const char* SharedBuffer::platformData() const
{
    return (const char*)CFDataGetBytePtr(m_cfData.get());
}

unsigned SharedBuffer::platformDataSize() const
{
    return CFDataGetLength(m_cfData.get());
}

void SharedBuffer::maybeTransferPlatformData()
{
    if (!m_cfData)
        return;
    
    ASSERT(!m_size);
        
    append((const char*)CFDataGetBytePtr(m_cfData.get()), CFDataGetLength(m_cfData.get()));
        
    m_cfData = 0;
}

void SharedBuffer::clearPlatformData()
{
    m_cfData = 0;
}

}
