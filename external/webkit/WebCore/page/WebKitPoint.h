

#ifndef WebKitPoint_h
#define WebKitPoint_h

#include <wtf/RefCounted.h>

namespace WebCore {

    class WebKitPoint : public RefCounted<WebKitPoint> {
    public:

        static PassRefPtr<WebKitPoint> create()
        {
            return adoptRef(new WebKitPoint());
        }
        static PassRefPtr<WebKitPoint> create(float x, float y)
        {
            return adoptRef(new WebKitPoint(x, y));
        }

        float x() const { return m_x; }
        float y() const { return m_y; }
        
        void setX(float x) { m_x = x; }
        void setY(float y) { m_y = y; }

    private:
        WebKitPoint(float x=0, float y=0)
            : m_x(x)
            , m_y(y)
        {
        }

        float m_x, m_y;
    };

} // namespace WebCore

#endif // WebKitPoint_h
