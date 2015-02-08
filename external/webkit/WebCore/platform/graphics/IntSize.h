

#ifndef IntSize_h
#define IntSize_h

#include <wtf/Platform.h>

#if PLATFORM(CG)
typedef struct CGSize CGSize;
#endif

#if PLATFORM(MAC)
#ifdef NSGEOMETRY_TYPES_SAME_AS_CGGEOMETRY_TYPES
typedef struct CGSize NSSize;
#else
typedef struct _NSSize NSSize;
#endif
#endif

#if PLATFORM(WIN)
typedef struct tagSIZE SIZE;
#elif PLATFORM(QT)
#include <qglobal.h>
QT_BEGIN_NAMESPACE
class QSize;
QT_END_NAMESPACE
#elif PLATFORM(HAIKU)
class BSize;
#endif

#if PLATFORM(WX)
class wxSize;
#endif

#if PLATFORM(BREWMP)
typedef struct AEESize AEESize;
#endif

namespace WebCore {

class IntSize {
public:
    IntSize() : m_width(0), m_height(0) { }
    IntSize(int width, int height) : m_width(width), m_height(height) { }
    
    int width() const { return m_width; }
    int height() const { return m_height; }

    void setWidth(int width) { m_width = width; }
    void setHeight(int height) { m_height = height; }

    bool isEmpty() const { return m_width <= 0 || m_height <= 0; }
    bool isZero() const { return !m_width && !m_height; }
    
    void expand(int width, int height)
    {
        m_width += width;
        m_height += height;
    }
    
    void scale(float scale)
    {
        m_width = static_cast<int>(static_cast<float>(m_width) * scale);
        m_height = static_cast<int>(static_cast<float>(m_height) * scale);
    }
    
    IntSize expandedTo(const IntSize& other) const
    {
        return IntSize(m_width > other.m_width ? m_width : other.m_width,
            m_height > other.m_height ? m_height : other.m_height);
    }

    IntSize shrunkTo(const IntSize& other) const
    {
        return IntSize(m_width < other.m_width ? m_width : other.m_width,
            m_height < other.m_height ? m_height : other.m_height);
    }

    void clampNegativeToZero()
    {
        *this = expandedTo(IntSize());
    }

#if PLATFORM(CG)
    explicit IntSize(const CGSize&); // don't do this implicitly since it's lossy
    operator CGSize() const;
#endif

#if PLATFORM(MAC) && !defined(NSGEOMETRY_TYPES_SAME_AS_CGGEOMETRY_TYPES)
    explicit IntSize(const NSSize &); // don't do this implicitly since it's lossy
    operator NSSize() const;
#endif

#if PLATFORM(WIN)
    IntSize(const SIZE&);
    operator SIZE() const;
#endif

#if PLATFORM(QT)
    IntSize(const QSize&);
    operator QSize() const;
#endif

#if PLATFORM(HAIKU)
    explicit IntSize(const BSize&);
    operator BSize() const;
#endif

#if PLATFORM(WX)
    IntSize(const wxSize&);
    operator wxSize() const;
#endif

#if PLATFORM(BREWMP)
    IntSize(const AEESize&);
    operator AEESize() const;
#endif

private:
    int m_width, m_height;
};

inline IntSize& operator+=(IntSize& a, const IntSize& b)
{
    a.setWidth(a.width() + b.width());
    a.setHeight(a.height() + b.height());
    return a;
}

inline IntSize& operator-=(IntSize& a, const IntSize& b)
{
    a.setWidth(a.width() - b.width());
    a.setHeight(a.height() - b.height());
    return a;
}

inline IntSize operator+(const IntSize& a, const IntSize& b)
{
    return IntSize(a.width() + b.width(), a.height() + b.height());
}

inline IntSize operator-(const IntSize& a, const IntSize& b)
{
    return IntSize(a.width() - b.width(), a.height() - b.height());
}

inline IntSize operator-(const IntSize& size)
{
    return IntSize(-size.width(), -size.height());
}

inline bool operator==(const IntSize& a, const IntSize& b)
{
    return a.width() == b.width() && a.height() == b.height();
}

inline bool operator!=(const IntSize& a, const IntSize& b)
{
    return a.width() != b.width() || a.height() != b.height();
}

} // namespace WebCore

#endif // IntSize_h
