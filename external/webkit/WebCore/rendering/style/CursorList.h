

#ifndef CursorList_h
#define CursorList_h

#include "CursorData.h"
#include <wtf/RefCounted.h>
#include <wtf/Vector.h>

namespace WebCore {

class CursorList : public RefCounted<CursorList> {
public:
    static PassRefPtr<CursorList> create()
    {
        return adoptRef(new CursorList);
    }

    const CursorData& operator[](int i) const { return m_vector[i]; }

    bool operator==(const CursorList& o) const { return m_vector == o.m_vector; }
    bool operator!=(const CursorList& o) const { return m_vector != o.m_vector; }

    size_t size() const { return m_vector.size(); }
    void append(const CursorData& cursorData) { m_vector.append(cursorData); }

private:
    CursorList()
    {
    }

    Vector<CursorData> m_vector;
};

} // namespace WebCore

#endif // CursorList_h
