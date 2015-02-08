

#ifndef ClientRect_h
#define ClientRect_h

#include "FloatRect.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class IntRect; 

    class ClientRect : public RefCounted<ClientRect> {
    public:
        static PassRefPtr<ClientRect> create() { return adoptRef(new ClientRect); }
        static PassRefPtr<ClientRect> create(const IntRect& rect) { return adoptRef(new ClientRect(rect)); }

        float top() const { return m_rect.y(); }
        float right() const { return m_rect.right(); }
        float bottom() const { return m_rect.bottom(); }
        float left() const { return m_rect.x(); }
        float width() const { return m_rect.width(); }
        float height() const { return m_rect.height(); }

    private:
        ClientRect();
        ClientRect(const IntRect&);

        FloatRect m_rect;
    }; 

} // namespace WebCore

#endif // ClientRect_h
