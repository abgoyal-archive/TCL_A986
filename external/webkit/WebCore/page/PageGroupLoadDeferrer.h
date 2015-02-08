

#ifndef PageGroupLoadDeferrer_h
#define PageGroupLoadDeferrer_h

#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

    class Frame;
    class Page;

    class PageGroupLoadDeferrer : public Noncopyable {
    public:
        PageGroupLoadDeferrer(Page*, bool deferSelf);
        ~PageGroupLoadDeferrer();

    private:
        Vector<RefPtr<Frame>, 16> m_deferredFrames;
    };
}

#endif // PageGroupLoadDeferrer_h
