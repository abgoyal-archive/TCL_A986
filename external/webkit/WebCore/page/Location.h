

#ifndef Location_h
#define Location_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class Frame;
    class KURL;
    class String;

    class Location : public RefCounted<Location> {
    public:
        static PassRefPtr<Location> create(Frame* frame) { return adoptRef(new Location(frame)); }

        Frame* frame() const { return m_frame; }
        void disconnectFrame();

        String href() const;

        // URI decomposition attributes
        String protocol() const;
        String host() const;
        String hostname() const;
        String port() const;
        String pathname() const;
        String search() const;
        String hash() const;

        String toString() const;

    private:
        Location(Frame*);

        const KURL& url() const;

        Frame* m_frame;
    };

} // namespace WebCore

#endif // Location_h
