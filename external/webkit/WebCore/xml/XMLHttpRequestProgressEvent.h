

#ifndef XMLHttpRequestProgressEvent_h
#define XMLHttpRequestProgressEvent_h

#include "ProgressEvent.h"

namespace WebCore {

    class XMLHttpRequestProgressEvent : public ProgressEvent {
    public:
        static PassRefPtr<XMLHttpRequestProgressEvent> create()
        {
            return adoptRef(new XMLHttpRequestProgressEvent);
        }
        static PassRefPtr<XMLHttpRequestProgressEvent> create(const AtomicString& type, bool lengthComputable = false, unsigned loaded = 0, unsigned total = 0)
        {
            return adoptRef(new XMLHttpRequestProgressEvent(type, lengthComputable, loaded, total));
        }

        virtual bool isXMLHttpRequestProgressEvent() const { return true; }

        // Those 2 methods are to be compatible with Firefox and are only a wrapper on top of the real implementation.
        unsigned position() const { return loaded(); }
        unsigned totalSize() const { return total(); }

    private:
        XMLHttpRequestProgressEvent() { }
        XMLHttpRequestProgressEvent(const AtomicString& type, bool lengthComputable, unsigned loaded, unsigned total)
            : ProgressEvent(type, lengthComputable, loaded, total)
        {
        }
    };

} // namespace WebCore

#endif // XMLHttpRequestProgressEvent_h
