


#ifndef Screen_h
#define Screen_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class Frame;

    class Screen : public RefCounted<Screen> {
    public:
        static PassRefPtr<Screen> create(Frame *frame) { return adoptRef(new Screen(frame)); }

        Frame* frame() const;
        void disconnectFrame();

        unsigned height() const;
        unsigned width() const;
        unsigned colorDepth() const;
        unsigned pixelDepth() const;
        unsigned availLeft() const;
        unsigned availTop() const;
        unsigned availHeight() const;
        unsigned availWidth() const;

    private:
        Screen(Frame*);
        
        Frame* m_frame;
    };

} // namespace WebCore

#endif // Screen_h
