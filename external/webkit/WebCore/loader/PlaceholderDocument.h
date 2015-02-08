

#ifndef PlaceholderDocument_h
#define PlaceholderDocument_h

#include "Document.h"

namespace WebCore {

class PlaceholderDocument : public Document {
public:
    static PassRefPtr<PlaceholderDocument> create(Frame* frame)
    {
        return adoptRef(new PlaceholderDocument(frame));
    }

    virtual void attach();

private:
    PlaceholderDocument(Frame* frame) : Document(frame, false, false) { }
};

} // namespace WebCore

#endif // PlaceholderDocument_h
