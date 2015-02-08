

#ifndef WebPasteboardHelper_h
#define WebPasteboardHelper_h

#import <WebCore/PasteboardHelper.h>

@class WebHTMLView;

class WebPasteboardHelper : public WebCore::PasteboardHelper
{
public:
    WebPasteboardHelper(WebHTMLView* view) : m_view(view) {}
    virtual WebCore::String urlFromPasteboard(const NSPasteboard*, WebCore::String* title) const;
    virtual WebCore::String plainTextFromPasteboard(const NSPasteboard*) const;
    virtual DOMDocumentFragment* fragmentFromPasteboard(const NSPasteboard*) const;
    virtual NSArray* insertablePasteboardTypes() const;
    private:
    WebHTMLView* m_view;
};

#endif
