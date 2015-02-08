

#ifndef OpenTypeUtilities_h
#define OpenTypeUtilities_h

#include "PlatformString.h"
#include <wtf/Forward.h>

namespace WebCore {

struct BigEndianUShort;
struct EOTPrefix;
class SharedBuffer;

#if OS(WINCE)
typedef unsigned __int8 UInt8;
#endif

struct EOTHeader {
    EOTHeader();

    size_t size() const { return m_buffer.size(); }
    const uint8_t* data() const { return m_buffer.data(); }

    EOTPrefix* prefix() { return reinterpret_cast<EOTPrefix*>(m_buffer.data()); }
    void updateEOTSize(size_t);
    void appendBigEndianString(const BigEndianUShort*, unsigned short length);
    void appendPaddingShort();

private:
    Vector<uint8_t, 512> m_buffer;
};

bool getEOTHeader(SharedBuffer* fontData, EOTHeader& eotHeader, size_t& overlayDst, size_t& overlaySrc, size_t& overlayLength);
HANDLE renameAndActivateFont(SharedBuffer*, const String&);

} // namespace WebCore

#endif // OpenTypeUtilities_h
