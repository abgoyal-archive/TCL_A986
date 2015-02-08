

#ifndef BidiContext_h
#define BidiContext_h

#include <wtf/Assertions.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>
#include <wtf/unicode/Unicode.h>

namespace WebCore {

// Used to keep track of explicit embeddings.
class BidiContext : public RefCounted<BidiContext> {
public:
    static PassRefPtr<BidiContext> create(unsigned char level, WTF::Unicode::Direction direction, bool override = false, BidiContext* parent = 0);

    BidiContext* parent() const { return m_parent.get(); }
    unsigned char level() const { return m_level; }
    WTF::Unicode::Direction dir() const { return static_cast<WTF::Unicode::Direction>(m_direction); }
    bool override() const { return m_override; }

private:
    BidiContext(unsigned char level, WTF::Unicode::Direction direction, bool override, BidiContext* parent)
        : m_level(level)
        , m_direction(direction)
        , m_override(override)
        , m_parent(parent)
    {
    }

    unsigned char m_level;
    unsigned m_direction : 5; // Direction
    bool m_override : 1;
    RefPtr<BidiContext> m_parent;
};

bool operator==(const BidiContext&, const BidiContext&);

} // namespace WebCore

#endif // BidiContext_h
