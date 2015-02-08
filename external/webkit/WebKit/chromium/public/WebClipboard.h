

#ifndef WebClipboard_h
#define WebClipboard_h

#include "WebCommon.h"

namespace WebKit {

class WebImage;
class WebString;
class WebURL;

class WebClipboard {
public:
    enum Format {
        FormatHTML,
        FormatBookmark,
        FormatSmartPaste
    };

    enum Buffer {
        BufferStandard,
        // Used on platforms like the X Window System that treat selection
        // as a type of clipboard.
        BufferSelection
    };

    virtual bool isFormatAvailable(Format, Buffer) = 0;

    virtual WebString readPlainText(Buffer) = 0;
    virtual WebString readHTML(Buffer, WebURL*) = 0;

    virtual void writePlainText(const WebString&) = 0;
    virtual void writeHTML(
        const WebString& htmlText, const WebURL&,
        const WebString& plainText, bool writeSmartPaste) = 0;
    virtual void writeURL(
        const WebURL&, const WebString& title) = 0;
    virtual void writeImage(
        const WebImage&, const WebURL&, const WebString& title) = 0;

protected:
    ~WebClipboard() {}
};

} // namespace WebKit

#endif
