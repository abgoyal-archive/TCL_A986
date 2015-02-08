

#ifndef PasteboardPrivate_h
#define PasteboardPrivate_h

namespace WebCore {

    class PasteboardPrivate
    {
    public:
        enum ClipboardFormat {
            HTMLFormat,
            BookmarkFormat,
            WebSmartPasteFormat,
        };
        enum ClipboardBuffer {
            StandardBuffer,
            SelectionBuffer,
        };
    };

} // namespace WebCore

#endif
