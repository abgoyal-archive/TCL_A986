

#ifndef Media_h
#define Media_h

#include "DOMWindow.h"

namespace WebCore {

class Media : public RefCounted<Media> {
public:
    static PassRefPtr<Media> create(Frame* frame)
    {
        return adoptRef(new Media(frame));
    }

    void disconnectFrame() { m_frame = 0; }

    String type() const;

    bool matchMedium(const String&) const;
    
private:
    Media(Frame*);

    Frame* m_frame;
};

} // namespace

#endif // Media_h
