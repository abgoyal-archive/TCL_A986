

#ifndef BarInfo_h
#define BarInfo_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class Frame;

    class BarInfo : public RefCounted<BarInfo> {
    public:
        enum Type { Locationbar, Menubar, Personalbar, Scrollbars, Statusbar, Toolbar };

        static PassRefPtr<BarInfo> create(Frame* frame, Type type) { return adoptRef(new BarInfo(frame, type)); }

        Frame* frame() const;
        void disconnectFrame();

        Type type() const;

        bool visible() const;

    private:
        BarInfo(Frame*, Type);
        Frame* m_frame;
        Type m_type;
    };

} // namespace WebCore

#endif // BarInfo_h
