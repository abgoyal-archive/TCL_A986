

#ifndef PasteboardHelper_h
#define PasteboardHelper_h

#if PLATFORM(MAC)

#import <wtf/Forward.h>

#ifdef __OBJC__
@class DOMDocumentFragment;
#else
class DOMDocumentFragment;
#endif

namespace WebCore {

    class String;
    class Document;
    
    class PasteboardHelper {
    public:
        virtual ~PasteboardHelper() {}
        virtual String urlFromPasteboard(const NSPasteboard*, String* title) const = 0;
        virtual String plainTextFromPasteboard(const NSPasteboard*) const = 0;
        virtual DOMDocumentFragment* fragmentFromPasteboard(const NSPasteboard*) const = 0;
        virtual NSArray* insertablePasteboardTypes() const = 0;
    };
    
}
#endif // PLATFORM(MAC)

#endif // !PasteboardHelper_h
