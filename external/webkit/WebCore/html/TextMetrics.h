

#ifndef TextMetrics_h
#define TextMetrics_h

#include <wtf/RefCounted.h>

namespace WebCore {

class TextMetrics : public RefCounted<TextMetrics> {
public:
    static PassRefPtr<TextMetrics> create() { return adoptRef(new TextMetrics); }

    unsigned width() const { return m_width; }
    void setWidth(float w) { m_width = w; }

private:
    TextMetrics()
        : m_width(0)
    { }

    float m_width;
};

} // namespace WebCore

#endif // TextMetrics_h
